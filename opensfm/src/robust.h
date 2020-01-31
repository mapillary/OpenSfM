#pragma once

#include <algorithm>
#include <Eigen/Eigen>


struct Pose{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
};

template <class T, int N>
class Model {
  public:
  static const int SIZE = N;
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

class RelativePose : public Model<RelativePose, 2> {
 public:
  using MODEL = Pose;
  using DATA = std::pair<Eigen::Vector2d, Eigen::Vector2d>;
  static const int MINIMAL_SAMPLES = 5;

  template <class IT, class MODEL>
  static bool Model(IT begin, IT end, MODEL* model){
    return true;
  }

  template <class MODEL, class DATA>
  static double Error(const MODEL& model, const DATA& d){
    return 0;
  }

};

class Line : public Model<Line, 1> {
 public:
  using MODEL = Eigen::Vector2d;
  using DATA = Eigen::Vector2d;
  static const int MINIMAL_SAMPLES = 2;

  template <class IT, class MODEL>
  static bool Model(IT begin, IT end, MODEL* model){
    const auto x1 = *begin;
    const auto x2 = *(++begin);
    const auto b = (x1[0]*x2[1] - x1[1]*x2[0])/(x1[0]-x2[0]);
    const auto a = (x1[1] - b)/x1[0];
    *model << a, b;
    return true;
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

struct ScoreInfo {
  double score{0};
  std::vector<int> inliers_indices;
  std::map<std::string, double> scorer_specifics;

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
  
   template <class IT>
   ScoreInfo Score(IT begin, IT end) {
     ScoreInfo score;
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

  template <class IT>
  ScoreInfo Score(IT begin, IT end) {
    ScoreInfo score;
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

  template <class IT>
  ScoreInfo Score(IT begin, IT end) {
    const auto median = this->ComputeMedian(begin, end);
    const auto mad = 1.4826 * median;
    const auto threshold = this->multiplier_ * mad;
    ScoreInfo score;
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

  ScoreInfo Estimate(){
    ScoreInfo best_score;
    for( int i = 0; i < params_.iterations; ++i){
      const auto random_samples = GetRandomSamples();
      typename MODEL::MODEL current_model;
      if(MODEL::Model(random_samples.begin(), random_samples.end(), &current_model)){
        auto errors = MODEL::Errors(
            current_model, samples_.begin(), samples_.end());
        const auto score = scorer_.Score(errors.begin(), errors.end());
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
ScoreInfo RANSACLine(const Eigen::Matrix<double, -1, 2>& points,
                     double parameter, const RansacType& ransac_type) {
  std::vector<Line::DATA> samples(points.rows());
  for (int i = 0; i < points.rows(); ++i) {
    samples[i] = points.row(i);
  }

  RobustEstimatorParams params;
  switch(ransac_type){
    case RANSAC:
    {
      RansacScoring scorer(parameter);
      RobustEstimator<RansacScoring, Line> ransac(samples, scorer, params);
      return ransac.Estimate();
    }
    case MSAC:
    {
      MSacScoring scorer(parameter);
      RobustEstimator<MSacScoring, Line> ransac(samples, scorer, params);
      return ransac.Estimate();
    }
    case LMedS:
    {
      LMedSScoring scorer(parameter);
      RobustEstimator<LMedSScoring, Line> ransac(samples, scorer, params);
      return ransac.Estimate();
    }
  }
}
}  // namespace csfm