#include <map/tracks_manager.h>
#include <sfm/retriangulation.h>

#include <limits>

namespace sfm {
namespace retriangulation {
void RealignPoints(const map::Map& reference,
                   const map::TracksManager& tracks_manager,
                   map::Map& to_align) {
  const auto& all_reference_shots = reference.GetShots();
  const auto& to_align_shots = to_align.GetShots();
  constexpr auto max_dbl = std::numeric_limits<double>::max();

  for (auto& lm : to_align.GetLandmarks()) {
    const auto point = lm.second.GetGlobalPos();
    std::pair<double, map::ShotId> best_shot = std::make_pair(max_dbl, "");
    for (const auto& shot_n_obs :
         tracks_manager.GetTrackObservations(lm.first)) {
      const auto shot_id = shot_n_obs.first;
      if ((to_align_shots.find(shot_id) == to_align_shots.end()) ||
          (all_reference_shots.find(shot_id) == all_reference_shots.end())) {
        continue;
      }
      const auto& align_shot = to_align_shots.at(shot_id);
      const Vec3d ray = point - align_shot.GetPose()->GetOrigin();
      const double dist2 = ray.squaredNorm();
      if (dist2 < best_shot.first) {
        best_shot = std::make_pair(dist2, shot_id);
      }
    }

    if (best_shot.first == max_dbl) {
      continue;
    }
    const auto reference_shot = best_shot.second;

    const auto& shot_before = to_align_shots.at(reference_shot);
    const auto& shot_after = all_reference_shots.at(reference_shot);

    const Mat3d R = shot_after.GetPose()->RotationCameraToWorld() *
                    shot_before.GetPose()->RotationWorldToCamera();
    const double s = shot_before.scale != 0. ? (1.0 / shot_before.scale) : 1.0;

    const auto oc_before = shot_before.GetPose()->GetOrigin();
    const auto oc_after = shot_after.GetPose()->GetOrigin();
    lm.second.SetGlobalPos(s * R * (point - oc_before) + oc_after);
  }
}
}  // namespace retriangulation
}  // namespace sfm
