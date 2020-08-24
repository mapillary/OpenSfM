#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <random>

#include "random_sampler.h"
#include "scorer.h"

struct RobustEstimatorParams {
  int iterations{100};
  double probability{0.99};
  bool use_local_optimization{true};
  bool use_iteration_reduction{true};
  int local_optimization_iterations{10};

  RobustEstimatorParams() = default;
};

template <class MODEL>
bool ShouldStop(const RobustEstimatorParams& params,
                const ScoreInfo<typename MODEL::Type>& best_score,
                int samples_count, int iteration) {
  if (!params.use_iteration_reduction) {
    return false;
  }
  const double inliers_ratio =
      double(best_score.inliers_indices.size()) / samples_count;
  const double proba_one_outlier =
      std::min(1.0 - std::numeric_limits<double>::epsilon(),
               1.0 - std::pow(inliers_ratio, double(MODEL::MINIMAL_SAMPLES)));
  const auto max_iterations =
      std::log(1.0 - params.probability) / std::log(proba_one_outlier);
  return max_iterations < iteration;
}

template <class SCORING, class MODEL>
ScoreInfo<typename MODEL::Type> Estimate(
    const std::vector<typename MODEL::Data>& samples, const SCORING& scorer,
    const RobustEstimatorParams& params) {
  // For now, we use this default one, we could be extended to PROSAC sampling
  RandomSamplesGenerator<std::mt19937> random_generator;

  ScoreInfo<typename MODEL::Type> best_score;
  bool should_stop = false;
  for (int i = 0; i < params.iterations && !should_stop; ++i) {
    // Generate and compute some models
    const auto random_samples = random_generator.GetRandomSamples<MODEL>(
        samples, MODEL::MINIMAL_SAMPLES);
    typename MODEL::Type models[MODEL::MAX_MODELS];
    const auto models_count = MODEL::Estimate(random_samples.begin(),
                                              random_samples.end(), &models[0]);

    // Compute model's errors for each generated model
    for (int j = 0; j < models_count && !should_stop; ++j) {
      auto errors =
          MODEL::EvaluateModel(models[j], samples.begin(), samples.end());

      // Compute score based on errors
      ScoreInfo<typename MODEL::Type> score =
          scorer.Score(errors.begin(), errors.end(), best_score);
      score.model = models[j];
      score.lo_model = models[j];

      // Keep the best score (bigger, the better)
      best_score = std::max(score, best_score);
      const bool best_found =
          score.score == best_score.score &&
          score.inliers_indices.size() >= MODEL::MINIMAL_SAMPLES;

      // Run local optimization (inner non-minimal RANSAC on inliers)
      if (best_found && params.use_local_optimization) {
        for (int k = 0; k < params.local_optimization_iterations; ++k) {
          // Gather inliers and use them for getting random sample
          std::vector<typename MODEL::Data> inliers_samples;
          for (const auto idx : best_score.inliers_indices) {
            inliers_samples.push_back(samples[idx]);
          }

          // Same as Matas papers : min(inliers/2, 12)
          const int lo_sample_size_clamp = 12;
          const int lo_sample_size =
              std::max(std::min(lo_sample_size_clamp,
                                int(best_score.inliers_indices.size() * 0.5)),
                       MODEL::MINIMAL_SAMPLES);

          const auto lo_random_samples =
              random_generator.GetRandomSamples<MODEL>(inliers_samples,
                                                       lo_sample_size);

          typename MODEL::Type lo_models[MODEL::MAX_MODELS];
          const auto lo_models_count =
              MODEL::EstimateNonMinimal(lo_random_samples.begin(),
                                        lo_random_samples.end(), &lo_models[0]);
          for (int l = 0; l < lo_models_count; ++l) {
            // Compute LO model's errors on all samples
            auto lo_errors = MODEL::EvaluateModel(lo_models[l], samples.begin(),
                                                  samples.end());

            // Compute LO score based on errors
            ScoreInfo<typename MODEL::Type> lo_score =
                scorer.Score(lo_errors.begin(), lo_errors.end(), best_score);
            lo_score.model = best_score.model;
            lo_score.lo_model = lo_models[l];

            // Keep the best score (bigger, the better)
            best_score = std::max(lo_score, best_score);
          }
        }
      }

      // Based on actual inliers ratio, we might stop here
      should_stop = ShouldStop<MODEL>(params, best_score, samples.size(), i);
    }
  }
  return best_score;
}

enum RansacType { RANSAC = 0, MSAC = 1, LMedS = 2 };

template <class MODEL>
ScoreInfo<typename MODEL::Type> RunEstimation(
    const std::vector<typename MODEL::Data>& samples, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type) {
  const double model_threshold = MODEL::ThresholdAdapter(threshold);
  switch (ransac_type) {
    case RANSAC: {
      RansacScoring scorer(model_threshold);
      return Estimate<RansacScoring, MODEL>(samples, scorer, parameters);
    }
    case MSAC: {
      MSacScoring scorer(model_threshold);
      return Estimate<MSacScoring, MODEL>(samples, scorer, parameters);
    }
    case LMedS: {
      LMedSScoring scorer(model_threshold);
      return Estimate<LMedSScoring, MODEL>(samples, scorer, parameters);
    }
    default:
      throw std::runtime_error("Unsupported RANSAC type.");
  }
}
