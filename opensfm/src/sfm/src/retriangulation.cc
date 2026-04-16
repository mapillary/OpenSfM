#include <map/defines.h>
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
  for (const auto& [shot_to_id, shot_to_data] : map_to.GetShots()) {
    if (!map_from.HasShot(shot_to_id)) {
      continue;
    }
    const auto& shot_from = map_from.GetShot(shot_to_id);
    auto shot_from_pose = *shot_from.GetPose();
    const auto* shot_to_pose = shot_to_data.GetPose();

    // put 'from' in LLA of 'to'
    shot_from_pose.SetOrigin(shot_from_pose.GetOrigin() + from_to_offset);

    // store the transform that map relative position in 'from' to 'to' :
    //
    // X_to' = 1 / scale_from * Rcw_from * Rwc_to * ( X_to - Oc_to ) +
    // Oc_from_to
    //

    const double scale = shot_from.scale != 0. ? (1.0 / shot_from.scale) : 1.0;
    const Mat3d R_to_from = shot_from_pose.RotationCameraToWorld() *
                            shot_to_pose->RotationWorldToCamera();
    const Vec3d t_from_to = -scale * R_to_from * shot_to_pose->GetOrigin() +
                            shot_from_pose.GetOrigin();

    from_to_transforms[shot_to_id] =
        geometry::Similarity(R_to_from, t_from_to, scale);
  }

  // remap points of 'to' using the computed transforms if needed
  if (update_points) {
    constexpr auto max_dbl = std::numeric_limits<double>::max();
    for (auto& [lm_id, lm] : map_to.GetLandmarks()) {
      const auto point = lm.GetGlobalPos();
      std::pair<double, map::ShotId> best_shot = std::make_pair(max_dbl, "");
      for (const auto& [shot, obs] : lm.GetObservations()) {
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
      lm.SetGlobalPos(transform.Transform(lm.GetGlobalPos()));
    }
  }

  // finally, map shots and instances
  std::unordered_set<map::ShotId> to_delete;
  for (auto& [shot_to_id, shot_to_data] : map_to.GetShots()) {
    // remember any shot not in 'from' but in 'to' for further deletion
    if (!map_from.HasShot(shot_to_id)) {
      to_delete.insert(shot_to_id);
      continue;
    }

    // copy cameras and some metadata
    const auto& shot_from = map_from.GetShot(shot_to_id);
    auto& camera_to = map_to.GetCamera(shot_to_data.GetCamera()->id);
    camera_to.SetParametersValues(shot_from.GetCamera()->GetParametersValues());

    shot_to_data.scale = shot_from.scale;
    shot_to_data.merge_cc = shot_from.merge_cc;
  }

  // only map rig instances (assuming rig camera didn't change)
  for (auto& [instance_id, rig_instance_to] : map_to.GetRigInstances()) {
    for (const auto& [any_shot_id, any_shot] : rig_instance_to.GetShots()) {
      if (map_from_shots.find(any_shot_id) != map_from_shots.end()) {
        const auto& any_shot_from = map_from_shots.at(any_shot_id);
        auto& to_pose = rig_instance_to.GetPose();

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
}  // namespace sfm::retriangulation
