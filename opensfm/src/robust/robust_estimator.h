#pragma once

#include <algorithm>
#include <Eigen/Eigen>

#include "scorer.h"
#include "random_sampler.h"


struct RobustEstimatorParams{
  int iterations{100};
  double probability{0.99};
  bool use_local_optimization{true};
  bool use_iteration_reduction{true};
  int local_optimization_iterations{10};

  RobustEstimatorParams() = default;
};

template<class SCORING, class MODEL>
class RobustEstimator{
  public:
   RobustEstimator(const std::vector<typename MODEL::DATA>& samples,
                   const SCORING scorer, const RobustEstimatorParams& params)
       : samples_(samples), scorer_(scorer), params_(params) {}

   std::vector<typename MODEL::DATA> GetRandomSamples(const std::vector<typename MODEL::DATA>& samples, int size) {
     const auto random_sample_indices = random_generator_.Generate(
         size, samples.size() - 1);

     std::vector<typename MODEL::DATA> random_samples;
     std::for_each(random_sample_indices.begin(), random_sample_indices.end(),
                   [&random_samples, &samples](const int idx) {
                     random_samples.push_back(samples[idx]);
                   });
     return random_samples;
  };

  ScoreInfo<typename MODEL::MODEL> Estimate(){
    ScoreInfo<typename MODEL::MODEL> best_score;
    bool should_stop = false;
    for( int i = 0; i < params_.iterations && !should_stop; ++i){

      // Generate and compute some models
      const auto random_samples = GetRandomSamples(samples_, MODEL::MINIMAL_SAMPLES);
      typename MODEL::MODEL models[MODEL::MAX_MODELS];
      const auto models_count = MODEL::Model(random_samples.begin(), random_samples.end(), &models[0]);
      for(int j = 0; j < models_count && !should_stop; ++j){

        // Compute model's errors
        auto errors = MODEL::Errors(
            models[j], samples_.begin(), samples_.end());

        // Compute score based on errors
        ScoreInfo<typename MODEL::MODEL> score = scorer_.Score(errors.begin(), errors.end(), best_score);
        score.model = models[j];
        score.lo_model = models[j];
  
        // Keep the best score (bigger, the better)
        best_score = std::max(score, best_score);
        const bool best_found = score.score == best_score.score;

        // Run local optimization (inner non-minimal RANSAC on inliers)
        if(best_found && params_.use_local_optimization ){
          for(int k = 0; k < params_.local_optimization_iterations; ++k)
          { 
            // Gather inliers and use them for getting random sample
            std::vector<typename MODEL::DATA> inliers_samples;
            std::for_each(best_score.inliers_indices.begin(), best_score.inliers_indices.end(),
                    [&inliers_samples, this](const int idx) {
                      inliers_samples.push_back(samples_[idx]);
                    });

            // Same as Matas papers : min(inliers/2, 12)
            const int lo_sample_size_clamp = 12;
            const int lo_sample_size = std::min(lo_sample_size_clamp, int(best_score.inliers_indices.size()*0.5));
            const auto lo_random_samples = GetRandomSamples(inliers_samples, lo_sample_size);

            // The local model (LO) can be different
            using LOMODEL = typename MODEL::LOMODEL;

            // So far, we assume a single model
            typename LOMODEL::MODEL lo_models[MODEL::MAX_MODELS];   // TODO : use LOMODEL's specific MAX_MODELS
            const auto lo_models_count = LOMODEL::Model(lo_random_samples.begin(), lo_random_samples.end(), &lo_models[0]);
            for(int l = 0; l < lo_models_count; ++l){
              // Compute LO model's errors on all samples
              auto lo_errors = LOMODEL::Errors(lo_models[l], samples_.begin(), samples_.end());

              // Compute LO score based on errors
              ScoreInfo<typename LOMODEL::MODEL> lo_score = scorer_.Score(lo_errors.begin(), lo_errors.end(), best_score);
              lo_score.model = best_score.model;
              lo_score.lo_model = lo_models[l];

              // Keep the best score (bigger, the better)
              best_score = std::max(lo_score, best_score);
            }
          }
        }

        // Based on actual inliers ratio, we might stop here
        should_stop = ShouldStop(best_score, i);
      }
    }
    return best_score;
  }

private:
 bool ShouldStop(const ScoreInfo<typename MODEL::MODEL>& best_score,
                 int iteration) {
   if(!params_.use_iteration_reduction){
     return false;
   }
   const double inliers_ratio = double(best_score.inliers_indices.size()) / samples_.size();
   const double proba_one_outlier =
       std::min(1.0 - std::numeric_limits<double>::epsilon(),
                1.0 - std::pow(inliers_ratio, double(MODEL::MINIMAL_SAMPLES)));
   const auto max_iterations = std::log(1.0 - params_.probability) / std::log(proba_one_outlier);
   return max_iterations < iteration;
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

template<class MODEL>
ScoreInfo<typename MODEL::MODEL> RunEstimation(const std::vector<typename MODEL::DATA>& samples, double threshold,
                               const RobustEstimatorParams& parameters,
                               const RansacType& ransac_type){
  switch(ransac_type){
    case RANSAC:
    {
      RansacScoring scorer(threshold);
      RobustEstimator<RansacScoring, MODEL> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
    case MSAC:
    {
      MSacScoring scorer(threshold);
      RobustEstimator<MSacScoring, MODEL> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
    case LMedS:
    {
      LMedSScoring scorer(threshold);
      RobustEstimator<LMedSScoring, MODEL> ransac(samples, scorer, parameters);
      return ransac.Estimate();
    }
    default:
      throw std::runtime_error("Unsupported RANSAC type.");
  }
}