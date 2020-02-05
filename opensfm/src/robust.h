#pragma once

#include <algorithm>
#include <Eigen/Eigen>

#include "numeric.h"
#include "essential.h"
#include "pose.h"

template <class T, int N, int M>
class Model {
 public:
  static const int SIZE = N;
  static const int MAX_MODELS = M;
  using ERROR = Eigen::Matrix<double, SIZE, 1>;

  template <class IT, class MODEL>
  static std::vector<ERROR> Errors(const MODEL& model, IT begin, IT end) {
    std::vector<ERROR> errors;
    std::for_each(begin, end, [&errors, &model](const typename T::DATA& d) {
      errors.push_back(T::Error(model, d));
    });
    return errors;
  }
};

// See "Structure from Motion using full spherical panoramic cameras"
// for a nice summary of ray-based errors for SfM
class EpipolarSymmetric{
  public:
    static double Error(const Eigen::Matrix3d& model, const Eigen::Vector3d& x, const Eigen::Vector3d& y){
    Eigen::Vector3d E_x = model * x;
    Eigen::Vector3d Et_y = model.transpose() * y;
    return SQUARE(y.dot(E_x)) * ( 1 / E_x.head<2>().squaredNorm()
                                + 1 / Et_y.head<2>().squaredNorm())
      / 4.0;  // The divide by 4 is to make this match the sampson distance
  }
};


class EpipolarGeodesic{
  public:
    static double Error(const Eigen::Matrix3d& model, const Eigen::Vector3d& x, const Eigen::Vector3d& y){
    const double yt_E_x = y.dot(model * x);
    return std::asin(yt_E_x);
  }
};

template< class E = EpipolarSymmetric >
class EssentialMatrix : public Model<EssentialMatrix<E>, 1, 10> {
 public:
  using ERROR = typename Model<EssentialMatrix<E>, 1, 10>::ERROR;
  using MODEL = Eigen::Matrix3d;
  using DATA = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 5;

  template <class IT>
  static int Model(IT begin, IT end, MODEL* models){
    const auto essentials = EssentialFivePoints(begin, end);
    for(int i = 0; i < essentials.size(); ++i){
      models[i] = essentials[i];
    }
    return essentials.size();
  }

  template <class MODEL, class DATA>
  static ERROR Error(const MODEL& model, const DATA& d){
    const auto x = d.first;
    const auto y = d.second;
    ERROR e;
    e[0] = E::Error(model, x, y);
    return e;
  }
};

class Line : public Model<Line, 1, 1> {
 public:
  using MODEL = Eigen::Vector2d;
  using DATA = Eigen::Vector2d;
  static const int MINIMAL_SAMPLES = 2;

  template <class IT>
  static int Model(IT begin, IT end, MODEL* models){
    const auto x1 = *begin;
    const auto x2 = *(++begin);
    const auto b = (x1[0]*x2[1] - x1[1]*x2[0])/(x1[0]-x2[0]);
    const auto a = (x1[1] - b)/x1[0];
    models[0] << a, b;
    return 1;
  }

  template <class MODEL, class DATA>
  static ERROR Error(const MODEL& model, const DATA& d){
    const auto a = model[0];
    const auto b = model[1];
    ERROR e;
    e << d[1] - (a*d[0] + b);
    return e;
  }

};

template< class MODEL >
struct ScoreInfo {
  double score{0};
  std::vector<int> inliers_indices;
  MODEL model;

  friend bool operator<(const ScoreInfo& s1, const ScoreInfo& s2) {
    if (s1.score < s2.score) {
      return true;
    }
    return false;
  }
};

class RansacScoring{
  public:

   RansacScoring(double threshold):threshold_(threshold){}
  
   template <class IT, class T>
   ScoreInfo<T> Score(IT begin, IT end, const ScoreInfo<T>& best_score) {
     ScoreInfo<T>  score;
     for(IT it = begin; it != end; ++it){
       if (it->norm() < threshold_) {
         ++score.score;
         score.inliers_indices.push_back(int(it-begin));
       }
     }
     return score;
   }

   double threshold_{0};
};

class MedianBasedScoring {
 public:
  MedianBasedScoring() = default;
  MedianBasedScoring(double nth) : nth_(nth) {}

  template <class IT>
  double ComputeMedian(IT begin, IT end) {
    const int count = (end - begin);
    const int median_index = count * nth_;
    std::vector<double> norms(count);
    for (IT it = begin; it != end; ++it) {
        norms[(it-begin)] = it->norm();
    }
    std::nth_element(norms.begin(), norms.begin() + median_index, norms.end());
    return norms[median_index];
  }

protected:
  double nth_{0.5};
};

class MSacScoring{
 public:
  MSacScoring(double threshold): threshold_(threshold){}

  template <class IT, class T>
  ScoreInfo<T>  Score(IT begin, IT end, const ScoreInfo<T>& best_score) {
    ScoreInfo<T>  score;
    for (IT it = begin; it != end; ++it) {
      const auto v = (*it).norm();
      const int index = int(it - begin);
      if (v <= threshold_) {
        score.score += v * v;
        score.inliers_indices.push_back(index);
      } else {
        score.score += threshold_ * threshold_;
      }
    }
    const double eps = 1e-8;
    score.score = 1.0 / (score.score+eps);
    return score;
  }
  double threshold_{0};
};

class LMedSScoring : public MedianBasedScoring{
 public:
  LMedSScoring(double multiplier)
      : MedianBasedScoring(0.5), multiplier_(multiplier) {}

  template <class IT, class T>
  ScoreInfo<T>  Score(IT begin, IT end, const ScoreInfo<T>& best_score) {
    const auto median = this->ComputeMedian(begin, end);
    const auto mad = 1.4826 * median;
    const auto threshold = this->multiplier_ * mad;
    ScoreInfo<T>  score;
     for(IT it = begin; it != end; ++it){
       const auto v = it->norm();
       if (v <= threshold) {
         score.inliers_indices.push_back(int(it-begin));
       }
     }
     const double eps = 1e-8;
     score.score = 1.0/(median+eps);
     return score;
  }

  double multiplier_;
};

class RandomSamplesGenerator{
  public:
    using RAND_GEN = std::mt19937;
    using DISTRIBUTION  = std::uniform_int_distribution<RAND_GEN::result_type>;

    RandomSamplesGenerator(int seed = 42): generator_(seed){}
    std::vector<int> Generate(int size, int range_max){
      return GenerateOneSample(size, range_max);
    }

  private:
    std::vector<int> GenerateOneSample(int size, int range_max){
      std::vector<int> indices(size);
      DISTRIBUTION distribution(0, range_max);
      for(int i = 0; i < size; ++i){
        indices[i] = distribution(generator_);
      }
      return indices;
    }

    RAND_GEN generator_;
};

struct RobustEstimatorParams{
  int iterations{100};
  double probability{0.99};

  RobustEstimatorParams() = default;
};

template<class SCORING, class MODEL>
class RobustEstimator{
  public:
   RobustEstimator(const std::vector<typename MODEL::DATA>& samples,
                   const SCORING scorer, const RobustEstimatorParams& params)
       : samples_(samples), scorer_(scorer), params_(params) {}

   std::vector<typename MODEL::DATA> GetRandomSamples() {
     const auto random_sample_indices = random_generator_.Generate(
         MODEL::MINIMAL_SAMPLES, samples_.size() - 1);

     std::vector<typename MODEL::DATA> random_samples;
     std::for_each(random_sample_indices.begin(), random_sample_indices.end(),
                   [&random_samples, this](const int idx) {
                     random_samples.push_back(samples_[idx]);
                   });
     return random_samples;
  };

  ScoreInfo<typename MODEL::MODEL> Estimate(){
    ScoreInfo<typename MODEL::MODEL> best_score;
    for( int i = 0; i < params_.iterations; ++i){
      const auto random_samples = GetRandomSamples();
      typename MODEL::MODEL models[MODEL::MAX_MODELS];
      const auto models_count = MODEL::Model(random_samples.begin(), random_samples.end(), &models[0]);
      for(int i = 0; i < models_count; ++i){
        auto errors = MODEL::Errors(
            models[i], samples_.begin(), samples_.end());
        ScoreInfo<typename MODEL::MODEL> score = scorer_.Score(errors.begin(), errors.end(), best_score);
        score.model = models[i];
        best_score = std::max(score, best_score);
      }
    }
    return best_score;
  }

  const std::vector<typename MODEL::DATA> samples_;
  SCORING scorer_;
  RobustEstimatorParams params_;
  RandomSamplesGenerator random_generator_;
};

enum RansacType{
  RANSAC = 0,
  MSAC = 1,
  LMedS = 2
};

namespace csfm {
ScoreInfo<Line::MODEL> RANSACLine(const Eigen::Matrix<double, -1, 2>& points,
                                  double threshold,
                                  const RobustEstimatorParams& parameters,
                                  const RansacType& ransac_type) {
  std::vector<Line::DATA> samples(points.rows());
  for (int i = 0; i < points.rows(); ++i) {
    samples[i] = points.row(i);
  }

  switch(ransac_type){
    case RANSAC:
    {
      RansacScoring scorer(threshold);
      RobustEstimator<RansacScoring, Line> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
    case MSAC:
    {
      MSacScoring scorer(threshold);
      RobustEstimator<MSacScoring, Line> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
    case LMedS:
    {
      LMedSScoring scorer(threshold);
      RobustEstimator<LMedSScoring, Line> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
  }
}

using EssentialMatrixModel = EssentialMatrix<EpipolarGeodesic>;
ScoreInfo<EssentialMatrixModel::MODEL> RANSACEssential(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters,
    const RansacType& ransac_type) {
  if((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())){
    throw std::runtime_error("Features matrices have different sizes.");
  }
  
  std::vector<EssentialMatrixModel::DATA> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }

  switch(ransac_type){
    case RANSAC:
    {
      RansacScoring scorer(threshold);
      RobustEstimator<RansacScoring, EssentialMatrixModel> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
    case MSAC:
    {
      MSacScoring scorer(threshold);
      RobustEstimator<MSacScoring, EssentialMatrixModel> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
    case LMedS:
    {
      LMedSScoring scorer(threshold);
      RobustEstimator<LMedSScoring, EssentialMatrixModel> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
  }
}

ScoreInfo<Eigen::Matrix<double, 3, 4>> RANSACRelativePose(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, 
    double threshold, const RobustEstimatorParams& parameters,
    const RansacType& ransac_type) {
  const auto essential_result = RANSACEssential(x1, x2, threshold, parameters, ransac_type);

  std::vector<EssentialMatrixModel::DATA> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }

  ScoreInfo<Eigen::Matrix<double, 3, 4>> relative_pose_result;
  relative_pose_result.inliers_indices = essential_result.inliers_indices;
  relative_pose_result.score = essential_result.score;
  relative_pose_result.model = ::RelativePoseFromEssential(essential_result.model, samples.begin(), samples.end());
  return relative_pose_result;
}

}  // namespace csfm