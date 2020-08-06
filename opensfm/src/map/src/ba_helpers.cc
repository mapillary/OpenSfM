#include <map/ba_helpers.h>

void BAHelpers::SetUpBAFromReconstruction(map::Map& map, BundleAdjuster& ba) {
  ba.SetUseAnalyticDerivatives(true);
  constexpr bool fix_cameras{true};
  // TODO: Check original vs optimized camera?
  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    ba.AddCamera(cam.id, cam, cam, fix_cameras);
  }

  for (const auto& pt_pair : map.GetAllLandmarks()) {
    const auto& pt = pt_pair.second;
    ba.AddPoint(pt.id_, pt.GetGlobalPos(), false);
  }
  constexpr bool bundle_use_gps{true};
  for (const auto& shot_pair : map.GetAllShots()) {
    const auto& shot = shot_pair.second;
    const auto& pose = shot.GetPose();
    ba.AddShot(shot.id_, shot.shot_camera_->id, pose.RotationWorldToCameraMin(),
               pose.TranslationWorldToCamera(), false);
    if (bundle_use_gps) {
      const Vec3d g = shot.shot_measurements_.gps_position_.Value();
      ba.AddPositionPrior(shot.id_, g[0], g[1], g[2],
                          shot.shot_measurements_.gps_accuracy_.Value());
    }
    // Now, divide betwen linear datastructure and map
    if (shot.UseLinearDataStructure()) {
        const auto& keypts = shot.GetKeyPoints();
        const auto& landmarks = shot.GetLandmarks();
        assert(keypts.size() == landmarks.size());
        for (size_t idx = 0; idx < landmarks.size(); ++idx)
        {
            const auto* lm = landmarks[idx];
            if (lm != nullptr)
            {
                //add
                const auto& obs = keypts[idx];
                ba.AddPointProjectionObservation(shot.id_, lm->id_, obs.point[0], obs.point[1], obs.scale);
            }
        }
    }
    else
    {
        for (const auto& lm_obs : shot.GetLandmarkObservations())
        {
            const auto& obs = lm_obs.second;
            ba.AddPointProjectionObservation(shot.id_, lm_obs.first->id_, obs.point[0], obs.point[1], obs.scale);
        }
    }
    
  }
}