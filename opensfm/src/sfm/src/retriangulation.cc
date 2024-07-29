#include <map/defines.h>
#include <map/tracks_manager.h>
#include <sfm/retriangulation.h>

#include <limits>
#include <unordered_set>

namespace sfm::retriangulation {
void RealignMaps(const map::Map& map_from, map::Map& map_to,
                 bool update_points) {
  const auto& map_from_shots = map_from.GetShots();

  const auto& from_ref = map_from.GetTopocentricConverter();
  const auto& to_ref = map_to.GetTopocentricConverter();
  const auto& from_to_offset = to_ref.ToTopocentric(from_ref.GetLlaRef());

  // first, record transforms that remap points of 'to'
  std::unordered_map<map::ShotId, geometry::Similarity> from_to_transforms;
  for (const auto& shot_to : map_to.GetShots()) {
    if (!map_from.HasShot(shot_to.first)) {
      continue;
    }
    const auto& shot_from = map_from.GetShot(shot_to.first);
    auto shot_from_pose = *shot_from.GetPose();
    const auto shot_to_pose = *shot_to.second.GetPose();

    // put 'from' in LLA of 'to'
    shot_from_pose.SetOrigin(shot_from_pose.GetOrigin() + from_to_offset);

    // store the transform that map relative position in 'from' to 'to' :
    //
    // X_to' = 1 / scale_from * Rcw_from * Rwc_to * ( X_to - Oc_to ) +
    // Oc_from_to
    //

    const double scale = shot_from.scale != 0. ? (1.0 / shot_from.scale) : 1.0;
    const Mat3d R_to_from = shot_from_pose.RotationCameraToWorld() *
                            shot_to_pose.RotationWorldToCamera();
    const Vec3d t_from_to = -scale * R_to_from * shot_to_pose.GetOrigin() +
                            shot_from_pose.GetOrigin();

    from_to_transforms[shot_to.first] =
        geometry::Similarity(R_to_from, t_from_to, scale);
  }

  // remap points of 'to' using the computed transforms if needed
  if (update_points) {
    constexpr auto max_dbl = std::numeric_limits<double>::max();
    for (auto& lm : map_to.GetLandmarks()) {
      const auto point = lm.second.GetGlobalPos();
      std::pair<double, map::ShotId> best_shot = std::make_pair(max_dbl, "");
      for (const auto& shot_n_obs : lm.second.GetObservations()) {
        const auto shot = shot_n_obs.first;
        if (map_from_shots.find(shot->GetId()) == map_from_shots.end()) {
          continue;
        }
        const Vec3d ray = point - shot->GetPose()->GetOrigin();
        const double dist2 = ray.squaredNorm();
        if (dist2 < best_shot.first) {
          best_shot = std::make_pair(dist2, shot->GetId());
        }
      }

      if (best_shot.first == max_dbl) {
        continue;
      }
      const auto reference_shot = best_shot.second;
      if (from_to_transforms.find(reference_shot) == from_to_transforms.end()) {
        continue;
      }
      const auto transform = from_to_transforms.at(reference_shot);
      lm.second.SetGlobalPos(transform.Transform(lm.second.GetGlobalPos()));
    }
  }

  // finally, map shots and instances
  std::unordered_set<map::ShotId> to_delete;
  for (auto& shot_to : map_to.GetShots()) {
    // remember any shot not in 'from' but in 'to' for further deletion
    if (!map_from.HasShot(shot_to.first)) {
      to_delete.insert(shot_to.first);
      continue;
    }

    // copy cameras and some metadata
    const auto& shot_from = map_from.GetShot(shot_to.first);
    auto& camera_to = map_to.GetCamera(shot_to.second.GetCamera()->id);
    camera_to.SetParametersValues(shot_from.GetCamera()->GetParametersValues());

    shot_to.second.scale = shot_from.scale;
    shot_to.second.merge_cc = shot_from.merge_cc;
  }

  // only map rig instances (assuming rig camera didn't change)
  for (auto& rig_instance_to : map_to.GetRigInstances()) {
    for (const auto& any_shot : rig_instance_to.second.GetShots()) {
      if (map_from_shots.find(any_shot.first) != map_from_shots.end()) {
        const auto& any_shot_from = map_from_shots.at(any_shot.first);
        auto& to_pose = rig_instance_to.second.GetPose();

        // assign 'from' pose
        to_pose = any_shot_from.GetRigInstance()->GetPose();

        // put 'from' to 'to' LLA
        to_pose.SetOrigin(to_pose.GetOrigin() + from_to_offset);
        break;
      }
    }
  }

  // delete any shot not in 'from' but in 'to'
  for (const auto& shot_id : to_delete) {
    map_to.RemoveShot(shot_id);
  }
}
}  // namespace sfm
