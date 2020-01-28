#pragma once

#include <algorithm>
#include <Eigen/Eigen>


struct Pose{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
};

template <class T>
class Model {
  static std::vector<double> Errors(const MODEL& model, IT begin, IT end) {
    std::vector<double> errors;
    std::for_each(begin, end, [&errors](const T::DATA& d) {
      errors.push_back(T::Error(model, d));
    });
    return errors;
  }
}

class RelativePose : public Model {
 public:
  using MODEL = Pose;
  using DATA = std::pair<Eigen::Vector2d, Eigen::Vector2d>;
  using MINIMAL_SAMPLES = 5;

  template <class IT>
  static bool Model(IT begin, IT end, MODEL* model){
    return 
  }

  static double Error(const MODEL& model, const DATA& d){
    return 0;
  }

};

struct ScoreInfo {
  double score{std::numeric_limits<double>::max()};
  std::vector<int> inliers_indices;

  friend bool operator<(const ScoreInfo& s){
    if(this->score < s.score){
      return bool;
    }
};

class RansacScoring{
  public:

   RansacScoring(double threshold):threshold_(threshold){}
  
   template <class IT>
   ScoreInfo Score(IT begin, IT end) {
     int index = 0;
     ScoreInfo score;
     score.score = 0;
     std::for_each(begin, end, [&below_threshold, &index](const double v) {
       if (v < threshold_) {
         ++score.below_threshold;
         score.inliers_indices.push_back(index);
       }
       ++index;
     });
     return score;
   }

   double threshold_{0};
};

double ComputeSigma(IT begin, IT end) {
  int median_index = (end - begin) * nth_;
  const auto median* nth_element(begin, nth_index, end);
  return 1.486 * median;
}

class MSacScoring : public AdaptiveScoring {
  public:

   RansacScoring(double multiplier):multiplier_(multiplier){}
  
   template <class IT>
   ScoreInfo Score(IT begin, IT end) {
    const auto threshold = multiplier_*ComputeSigma(begin, end);

     int index = 0;
     ScoreInfo score;
     score.score = 0;
     std::for_each(begin, end, [&below_threshold, &index](const double v) {
       if (v < threshold) {
         score.below_threshold += v*v;
         score.inliers_indices.push_back(index);
       }
       else{
         score.below_threshold += threshold*threshold;
       }
       ++index;
     });
     return score;
   }

   double multiplier_{2.5};
};

class LMedSScoring {
 public:
  LMedSScoring(double nth) : nth_(nth) {}

  template <class IT>
  ScoreInfo Score(IT begin, IT end) {
    int nth_index = (end - begin) * nth_;
    return *nth_element(begin, nth_index, end);
  }

  double nth_{0.5};
};

class RandomSamplesGenerator{
  public:
    static std::vector<int> Generate(int size){
      std::vector<int> samples;
      return samples;
    }
};

class RobustEstimator<SCORING, MODEL>{
  std::vector<MODEL::DATA> GetRandomSamples() {
    const auto random_sample_indices =
        RandomSamplesGenerator::Generate(MODEL::MINIMAL_SAMPLES);
    std::vector<MODEL::DATA> random_samples(MODEL::MINIMAL_SAMPLES);
    std::for_each(random_sample_indices.begin(), random_sample_indices.end(),
                  [&random_samples, &samples_](const int idx) {
                    random_samples.push_back(samples_[idx]);
                  });
    return random_samples;
  };

  bool Estimate(){
    for in i = 0; i < num_iterations; ++i){
      const auto random_samples = GetRandomSamples();
      MODEL current_model;
      if(MODEL::Model(samples, &current_model)){
        const auto all_errors = MODEL::Errors(
            current_model, all_samples.begin(), all_samples.end());
        const auto score = SCORING::Score(all_errors);
        best_score = max(score, best_score);
      }
    }
  }

  const std::vector<MODEL::DATA> samples_;
};