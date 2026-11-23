#pragma once

#include <map/map.h>

#include <unordered_map>

namespace sfm::mount_pitch {

struct Options {
  size_t min_samples{10};
  double max_deviation_deg{3.0};
  double max_abs_deg{30.0};
};

// Estimates, for each shot, the nose-up angle (radians) of its camera relative
// to the direction of travel of its sequence. Shots are grouped by sequence key
// and camera; shots without a sequence key get no entry. Each group gets the
// median of its per-shot samples, or no entry when the estimate is unreliable.
// Invariant to the reconstruction's similarity frame, so it can run before GPS
// alignment.
std::unordered_map<map::ShotId, double> EstimateMountPitches(
    const map::Map& map, const Options& options);

}  // namespace sfm::mount_pitch
