#include "sfm/mount_pitch.h"

#include <geometry/camera_instances.h>
#include <map/shot.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sfm::mount_pitch {

namespace {

// Steps much shorter than the group's typical step are stops, where the
// travel direction is dominated by pose noise.
constexpr double kMinStepRatio = 0.25;
// Travel must be within 60 degrees of the optical axis (or its opposite) for
// the pitch to be observable; side-looking cameras are skipped.
constexpr double kMinForwardCosine = 0.5;

using GroupKey = std::pair<std::string, map::CameraId>;

double Median(std::vector<double> values) {
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

bool IsEligible(const map::Shot& shot) {
  // Without a sequence key, shots of unrelated captures with different mounts
  // would be pooled into one estimate.
  const auto& measurements = shot.GetShotMeasurements();
  if (!measurements.capture_time_.HasValue() ||
      !measurements.sequence_key_.HasValue()) {
    return false;
  }
  if (measurements.orientation_.HasValue() &&
      measurements.orientation_.Value() != 1) {
    return false;
  }
  return shot.GetCamera()->GetProjectionType() !=
         geometry::ProjectionType::SPHERICAL;
}

std::map<GroupKey, std::vector<const map::Shot*>> GroupShots(
    const map::Map& map) {
  std::map<GroupKey, std::vector<const map::Shot*>> groups;
  for (const auto& [shot_id, shot] : map.GetShots()) {
    if (!IsEligible(shot)) {
      continue;
    }
    groups[{shot.GetShotMeasurements().sequence_key_.Value(),
            shot.GetCamera()->id}]
        .push_back(&shot);
  }
  for (auto& [key, shots] : groups) {
    std::sort(shots.begin(), shots.end(), [](const auto* a, const auto* b) {
      return a->GetShotMeasurements().capture_time_.Value() <
             b->GetShotMeasurements().capture_time_.Value();
    });
  }
  return groups;
}

std::vector<double> PitchSamples(const std::vector<const map::Shot*>& shots) {
  std::vector<Vec3d> steps;
  std::vector<double> step_norms;
  for (size_t i = 1; i + 1 < shots.size(); ++i) {
    steps.emplace_back(shots[i + 1]->GetPose()->GetOrigin() -
                       shots[i - 1]->GetPose()->GetOrigin());
    step_norms.push_back(steps.back().norm());
  }
  std::vector<double> samples;
  if (steps.empty()) {
    return samples;
  }
  const double min_step = kMinStepRatio * Median(step_norms);
  for (size_t i = 0; i < steps.size(); ++i) {
    if (step_norms[i] <= min_step) {
      continue;
    }
    Vec3d travel = shots[i + 1]->GetPose()->RotationWorldToCamera() * steps[i];
    if (travel.z() < 0) {
      travel = -travel;
    }
    if (travel.z() < kMinForwardCosine * travel.norm()) {
      continue;
    }
    // Image y points down, so travel below the optical axis means nose-up.
    samples.push_back(std::atan2(travel.y(), travel.z()));
  }
  return samples;
}

std::optional<double> EstimateGroupPitch(
    const std::vector<const map::Shot*>& shots, const Options& options) {
  const auto samples = PitchSamples(shots);
  if (samples.empty() || samples.size() < options.min_samples) {
    return std::nullopt;
  }
  const double median = Median(samples);
  std::vector<double> deviations;
  deviations.reserve(samples.size());
  for (const double sample : samples) {
    deviations.push_back(std::abs(sample - median));
  }
  constexpr double kDegToRad = M_PI / 180.0;
  if (Median(deviations) > options.max_deviation_deg * kDegToRad ||
      std::abs(median) > options.max_abs_deg * kDegToRad) {
    return std::nullopt;
  }
  return median;
}

}  // namespace

std::unordered_map<map::ShotId, double> EstimateMountPitches(
    const map::Map& map, const Options& options) {
  std::unordered_map<map::ShotId, double> pitches;
  for (const auto& [key, shots] : GroupShots(map)) {
    const auto pitch = EstimateGroupPitch(shots, options);
    if (!pitch) {
      continue;
    }
    for (const auto* shot : shots) {
      pitches[shot->GetId()] = *pitch;
    }
  }
  return pitches;
}

}  // namespace sfm::mount_pitch
