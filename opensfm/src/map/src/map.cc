#include <map/map.h>
#include <geometry/pose.h>
#include <map/landmark.h>
#include <map/shot.h>
#include <unordered_set>

namespace map {

void Map::AddObservation(Shot* const shot, Landmark* const lm,
                         const Observation& obs) {
  lm->AddObservation(shot, obs.id);
  shot->CreateObservation(lm, obs.point, obs.scale, obs.color, obs.id);
}

void Map::AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                         const Observation& obs) {
  auto* const shot = GetShot(shot_id);
  auto* const lm = GetLandmark(lm_id);
  AddObservation(shot, lm, obs);
}

void Map::RemoveObservation(const ShotId& shot_id, const LandmarkId& lm_id) {
  auto* shot = GetShot(shot_id);
  auto* lm = GetLandmark(lm_id);
  shot->RemoveLandmarkObservation(lm->GetObservationIdInShot(shot));
  lm->RemoveObservation(shot);
}

Shot* Map::GetShot(const ShotId& shot_id) {
  const auto& it = shots_.find(shot_id);
  if (it == shots_.end()) {
    throw std::runtime_error("Accessing invalid ShotID " + shot_id);
  }
  return &it->second;
}

Shot* Map::GetPanoShot(const ShotId& shot_id) {
  const auto& it = pano_shots_.find(shot_id);
  if (it == pano_shots_.end()) {
    throw std::runtime_error("Accessing invalid PanoShotID " + shot_id);
  }
  return &it->second;
}

Landmark* Map::GetLandmark(const LandmarkId& lm_id) {
  const auto& it = landmarks_.find(lm_id);
  if (it == landmarks_.end()) {
    throw std::runtime_error("Accessing invalid LandmarkId " + lm_id);
  }
  return &it->second;
}

void Map::ClearObservationsAndLandmarks() {
  // first JUST delete the observations of the landmark
  for (auto& id_lm : landmarks_) {
    auto& observations = id_lm.second.GetObservations();
    for (const auto& obs : observations) {
      obs.first->RemoveLandmarkObservation(obs.second);
    }
    id_lm.second.ClearObservations();
  }
  // then clear the landmarks_
  landmarks_.clear();
}

Shot* Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id) {
  return CreateShot(shot_id, camera_id, geometry::Pose());
}

/**
 * Creates a shot and returns a pointer to it
 *
 * @param shot_id       unique id of the shot
 * @param camera        previously created camera
 * @param global_pos    position in the 3D world
 *
 * @returns             returns pointer to created or existing shot
 */
Shot* Map::CreateShot(const ShotId& shot_id, const Camera* const cam,
                      const geometry::Pose& pose) {
  auto it_exist = shots_.find(shot_id);
  if (it_exist == shots_.end())  // create
  {
    auto it =
        shots_.emplace(std::piecewise_construct, std::forward_as_tuple(shot_id),
                       std::forward_as_tuple(shot_id, cam, pose));

    it.first->second.unique_id_ = shot_unique_id_;
    shot_unique_id_++;
    return &it.first->second;
  } else {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
}

/**
 * Creates a shot and returns a pointer to it
 *
 * @param shot_id       unique id of the shot
 * @param camera_id     unique id of EXISTING camera
 * @param global_pos    position in the 3D world
 *
 * @returns             returns pointer to created or existing shot
 */
Shot* Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                      const geometry::Pose& pose) {
  return CreateShot(shot_id, GetCamera(camera_id), pose);
}

void Map::RemoveShot(const ShotId& shot_id) {
  // 1) Find the point
  const auto& shot_it = shots_.find(shot_id);
  if (shot_it != shots_.end()) {
    auto& shot = shot_it->second;
    // 2) Remove it from all the points
    auto& lms_map = shot.GetLandmarkObservations();
    for (auto& lm_obs : lms_map) {
      lm_obs.first->RemoveObservation(&shot);
    }
    // 3) Remove from shots
    shots_.erase(shot_it);
  } else {
    throw std::runtime_error("Accessing invalid ShotID " + shot_id);
  }
}

Shot* Map::CreatePanoShot(const ShotId& shot_id, const CameraId& camera_id) {
  return CreatePanoShot(shot_id, camera_id, geometry::Pose());
}

/**
 * Creates a pano shot and returns a pointer to it
 *
 * @param shot_id       unique id of the shot
 * @param camera        previously created camera
 * @param global_pos    position in the 3D world
 *
 * @returns             returns pointer to created or existing shot
 */
Shot* Map::CreatePanoShot(const ShotId& shot_id, const Camera* const cam,
                          const geometry::Pose& pose) {
  auto it_exist = pano_shots_.find(shot_id);
  if (it_exist == pano_shots_.end()) {
    auto it = pano_shots_.emplace(std::piecewise_construct,
                                  std::forward_as_tuple(shot_id),
                                  std::forward_as_tuple(shot_id, cam, pose));
    it.first->second.unique_id_ = pano_shot_unique_id_;
    pano_shot_unique_id_++;
    return &(it.first->second);
  } else {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
}

Shot* Map::CreatePanoShot(const ShotId& shot_id, const CameraId& camera_id,
                          const geometry::Pose& pose) {
  return CreatePanoShot(shot_id, GetCamera(camera_id), pose);
}

void Map::RemovePanoShot(const ShotId& shot_id) {
  const auto& shot_it = pano_shots_.find(shot_id);
  if (shot_it != pano_shots_.end()) {
    const auto& shot = shot_it->second;
    pano_shots_.erase(shot_it);
  } else {
    throw std::runtime_error("Accessing invalid ShotID " + shot_id);
  }
}

/**
 * Creates a landmark and returns a pointer to it
 *
 * @param lm_Id       unique id of the landmark
 * @param global_pos  3D position of the landmark
 * @param name        name of the landmark
 *
 * @returns           pointer to the created or already existing lm
 */
Landmark* Map::CreateLandmark(
    const LandmarkId& lm_id,
    const Vec3d& global_pos)
{
  auto it_exist = landmarks_.find(lm_id);
  if (it_exist == landmarks_.end()) {
    auto it = landmarks_.emplace(std::piecewise_construct,
                                 std::forward_as_tuple(lm_id),
                                 std::forward_as_tuple(lm_id, global_pos));
    it.first->second.unique_id_ = landmark_unique_id_;
    landmark_unique_id_++;
    return &it.first->second;  // the raw pointer
  } else {
    throw std::runtime_error("Landmark " + lm_id + " already exists.");
  }
}

void Map::RemoveLandmark(const Landmark* const lm) {
  if (lm != nullptr) {
    RemoveLandmark(lm->id_);
  } else {
    throw std::runtime_error("Nullptr landmark");
  }
}

void Map::RemoveLandmark(const LandmarkId& lm_id) {
  // 1) Find the landmark
  const auto& lm_it = landmarks_.find(lm_id);
  if (lm_it != landmarks_.end()) {
    const auto& landmark = lm_it->second;

    // 2) Remove all its observation
    const auto& observations = landmark.GetObservations();
    for (const auto& obs : observations) {
      Shot* shot = obs.first;
      const auto feat_id = obs.second;
      shot->RemoveLandmarkObservation(feat_id);
    }

    // 3) Remove from landmarks
    landmarks_.erase(lm_it);
  } else {
    throw std::runtime_error("Accessing invalid LandmarkId " + lm_id);
  }
}

Camera* Map::CreateCamera(const Camera& cam) {
  auto make_cam = [](const Camera& cam) {
    switch (cam.GetProjectionType()) {
      case ProjectionType::PERSPECTIVE:
        return Camera::CreatePerspectiveCamera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::K1),
            cam.GetParameterValue(Camera::Parameters::K2));
      case ProjectionType::BROWN: {
        VecXd distortion(5);
        distortion << cam.GetParameterValue(Camera::Parameters::K1),
            cam.GetParameterValue(Camera::Parameters::K2),
            cam.GetParameterValue(Camera::Parameters::K3),
            cam.GetParameterValue(Camera::Parameters::P1),
            cam.GetParameterValue(Camera::Parameters::P2);
        return Camera::CreateBrownCamera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::AspectRatio),
            Vec2d(cam.GetParameterValue(Camera::Parameters::Cx),
                  cam.GetParameterValue(Camera::Parameters::Cy)),
            distortion);
      }
      case ProjectionType::FISHEYE:
        return Camera::CreateFisheyeCamera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::K1),
            cam.GetParameterValue(Camera::Parameters::K2));
      case ProjectionType::FISHEYE_OPENCV: {
        VecXd distortion(4);
        distortion << cam.GetParameterValue(Camera::Parameters::K1),
            cam.GetParameterValue(Camera::Parameters::K2),
            cam.GetParameterValue(Camera::Parameters::K3),
            cam.GetParameterValue(Camera::Parameters::K4);
        return Camera::CreateFisheyeOpencvCamera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::AspectRatio),
            Vec2d(cam.GetParameterValue(Camera::Parameters::Cx),
                  cam.GetParameterValue(Camera::Parameters::Cy)),
            distortion);
      }
      case ProjectionType::FISHEYE62: {
        VecXd distortion(8);
        distortion << cam.GetParameterValue(Camera::Parameters::K1),
            cam.GetParameterValue(Camera::Parameters::K2),
            cam.GetParameterValue(Camera::Parameters::K3),
            cam.GetParameterValue(Camera::Parameters::K4),
            cam.GetParameterValue(Camera::Parameters::K5),
            cam.GetParameterValue(Camera::Parameters::K6),
            cam.GetParameterValue(Camera::Parameters::P1),
            cam.GetParameterValue(Camera::Parameters::P2);
        return Camera::CreateFisheye62Camera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::AspectRatio),
            Vec2d(cam.GetParameterValue(Camera::Parameters::Cx),
                  cam.GetParameterValue(Camera::Parameters::Cy)),
            distortion);
      }
      case ProjectionType::RADIAL: {
        return Camera::CreateRadialCamera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::AspectRatio),
            Vec2d(cam.GetParameterValue(Camera::Parameters::Cx),
                  cam.GetParameterValue(Camera::Parameters::Cy)),
            Vec2d(cam.GetParameterValue(Camera::Parameters::K1),
                  cam.GetParameterValue(Camera::Parameters::K2)));
      }
      case ProjectionType::SIMPLE_RADIAL: {
        return Camera::CreateSimpleRadialCamera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::AspectRatio),
            Vec2d(cam.GetParameterValue(Camera::Parameters::Cx),
                  cam.GetParameterValue(Camera::Parameters::Cy)),
            cam.GetParameterValue(Camera::Parameters::K1));
      }
      case ProjectionType::DUAL:
        return Camera::CreateDualCamera(
            cam.GetParameterValue(Camera::Parameters::Transition),
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::K1),
            cam.GetParameterValue(Camera::Parameters::K2));
      case ProjectionType::SPHERICAL:
        return Camera::CreateSphericalCamera();
      default:
        throw std::runtime_error("Could not create unknown camera type: " +
                                 cam.id);
    }
  };
  auto it = cameras_.emplace(std::make_pair(cam.id, make_cam(cam)));
  auto& new_cam = it.first->second;
  new_cam.width = cam.width;
  new_cam.height = cam.height;
  new_cam.id = cam.id;
  return &new_cam;
}

Camera* Map::GetCamera(const CameraId& cam_id) {
  auto it = cameras_.find(cam_id);
  if (it == cameras_.end()) {
    throw std::runtime_error("Accessing invalid CameraId " + cam_id);
  }
  return &it->second;
}

Shot* Map::UpdateShot(const Shot& other_shot) {
  auto it_exist = shots_.find(other_shot.id_);
  if (it_exist == shots_.end()) {
    throw std::runtime_error("Shot " + other_shot.id_ + " does not exists.");
  } else {
    auto shot = &it_exist->second;
    shot->merge_cc = other_shot.merge_cc;
    shot->scale = other_shot.scale;
    shot->SetShotMeasurements(other_shot.shot_measurements_);
    shot->covariance = other_shot.covariance;
    shot->SetPose(other_shot.GetPose());
    return shot;
  }
}

Shot* Map::UpdatePanoShot(const Shot& other_shot) {
  auto it_exist = pano_shots_.find(other_shot.id_);
  if (it_exist == pano_shots_.end()) {
    throw std::runtime_error("Pano shot " + other_shot.id_ +
                             " does not exists.");
  } else {
    auto shot = &it_exist->second;
    shot->merge_cc = other_shot.merge_cc;
    shot->scale = other_shot.scale;
    shot->SetShotMeasurements(other_shot.shot_measurements_);
    shot->covariance = other_shot.covariance;
    shot->SetPose(other_shot.GetPose());
    return shot;
  }
}

};  // namespace map
