#include <map/landmark.h>
#include <map/map.h>
#include <map/pose.h>
#include <map/shot.h>

#include <unordered_set>

namespace map {

void Map::AddObservation(Shot* const shot, Landmark* const lm,
                         const FeatureId feat_id) {
  shot->AddLandmarkObservation(lm, feat_id);
  lm->AddObservation(shot, feat_id);
}

void Map::AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                         const FeatureId feat_id) {
  auto* const shot = GetShot(shot_id);
  if (shot == nullptr) {
    throw std::runtime_error("Accessing invalid ShotID " + shot_id);
  }
  auto& lm = landmarks_.at(lm_id);
  AddObservation(shot, lm.get(), feat_id);
}

void Map::AddObservation(Shot* const shot, Landmark* const lm,
                         const Observation& obs) {
  if (shot->UseLinearDataStructure())
  {
    AddObservation(shot, lm, obs.id);
  }
  else
  {
    shot->CreateObservation(lm, obs.point, obs.scale, obs.color, obs.id);
    lm->AddObservation(shot, obs.id);
  }
}

void Map::AddObservation(const ShotId& shot_id, const LandmarkId& lm_id, const Observation& obs)
{
  auto* const shot = GetShot(shot_id);
  if (shot == nullptr) {
    throw std::runtime_error("Accessing invalid ShotID " + shot_id);
  }
  auto* const lm = GetLandmark(lm_id);
  if (lm == nullptr)
  {
    throw std::runtime_error("Accessing invalid LandmarkID" + lm_id);
  }
  AddObservation(shot, lm, obs);
}

void Map::RemoveObservation(Shot* const shot, Landmark* const lm,
                            const FeatureId feat_id) {
  shot->RemoveLandmarkObservation(feat_id);
  lm->RemoveObservation(shot);
}

void Map::RemoveObservation(const ShotId& shot_id, const LandmarkId& lm_id)
{
  //get the shot
  auto* shot = GetShot(shot_id);
  //get the landmark
  auto* lm = GetLandmark(lm_id);
  if (shot != nullptr && lm != nullptr)
  {
    //get observation
    shot->RemoveLandmarkObservation(lm->GetObservationIdInShot(shot));
    //remove
    lm->RemoveObservation(shot);
  }
}

Shot* Map::GetShot(const ShotId& shot_id)  // const
{
  const auto& it = shots_.find(shot_id);
  return (it != shots_.end() ? it->second.get() : nullptr);
}

Landmark* Map::GetLandmark(const LandmarkId& lm_id) {
  const auto& it = landmarks_.find(lm_id);
  return (it != landmarks_.end() ? it->second.get() : nullptr);
}

void Map::ClearObservationsAndLandmarks() {
  // first JUST delete the observations of the landmark
  for (auto& id_lm : landmarks_) {
    auto& observations = id_lm.second->GetObservations();
    for (const auto& obs : observations) {
      obs.first->RemoveLandmarkObservation(obs.second);
    }
    id_lm.second->ClearObservations();
  }
  // then clear the landmarks_
  landmarks_.clear();
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
                      const Pose& pose) {
  // C++14
  // auto it = shots_.emplace(shot_id, std::make_unique<Shot>(shot_id, cam,
  // pose));
  // C++11
  auto it_exist = shots_.find(shot_id);
  if (it_exist == shots_.end())  // create
  {
    auto it = shots_.emplace(
        shot_id, std::unique_ptr<Shot>(new Shot(shot_id, cam, pose)));
    it.first->second->unique_id_ = shot_unique_id_;
    shot_unique_id_++;
    return it.first->second.get();
  } else {
    return it_exist->second.get();
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
                      const Pose& pose) {
  
  // auto* cam = ;
  return CreateShot(shot_id, GetCamera(camera_id), pose);
}

// void Map::UpdateShotPose(const ShotId& shot_id, const Pose& pose) {
//   shots_.at(shot_id)->SetPose(pose);
// }

void Map::RemoveShot(const ShotId& shot_id) {
  // 1) Find the point
  const auto& shot_it = shots_.find(shot_id);
  if (shot_it != shots_.end()) {
    const auto& shot = shot_it->second;
    // 2) Remove it from all the points
    for (const auto& lm : shot->GetLandmarks()) {
      if (lm != nullptr) {
        lm->RemoveObservation(shot.get());
      }
    }

    // 3) Remove from shots
    shots_.erase(shot_it);
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
    const Vec3d& global_pos)  //, const std::string& name)
{
  // C++14
  // auto it = landmarks_.emplace(lm_id, std::make_unique<Landmark>(lm_id,
  // global_pos));
  // C++11
  auto it_exist = landmarks_.find(lm_id);
  if (it_exist == landmarks_.end()) //create
  {
    auto it = landmarks_.emplace(
        lm_id, std::unique_ptr<Landmark>(new Landmark(lm_id, global_pos)));
    it.first->second->unique_id_ = landmark_unique_id_;
    landmark_unique_id_++;
    return it.first->second.get();  // the raw pointer
  }
  else
  {
    return it_exist->second.get();
  }
  
}

void Map::RemoveLandmark(const Landmark* const lm) {
  if (lm != nullptr) {
    // 1) Remove all its observation
    const auto& observations = lm->GetObservations();
    for (const auto& obs : observations) {
      Shot* shot = obs.first;
      const auto feat_id = obs.second;
      shot->RemoveLandmarkObservation(feat_id);
    }

    // 2) Remove from landmarks
    landmarks_.erase(lm->id_);
  }
}

void Map::RemoveLandmark(const LandmarkId& lm_id) {
  // 1) Find the landmark
  const auto& lm_it = landmarks_.find(lm_id);
  if (lm_it != landmarks_.end()) {
    const auto& landmark = lm_it->second;

    // 2) Remove all its observation
    const auto& observations = landmark->GetObservations();
    for (const auto& obs : observations) {
      Shot* shot = obs.first;
      const auto feat_id = obs.second;
      shot->RemoveLandmarkObservation(feat_id);
    }

    // 3) Remove from landmarks
    landmarks_.erase(lm_it);
  }
}

/**
 * Replaces landmark old_lm by new_lm
 *
 */

void Map::ReplaceLandmark(Landmark* old_lm, Landmark* new_lm) {
  if (old_lm == nullptr || new_lm == nullptr || old_lm->id_ == new_lm->id_) {
    return;
  }
  // go through the observations of old_lm
  for (const auto& observation : old_lm->GetObservations()) {
    Shot* obs_shot = observation.first;
    FeatureId obs_feat_id = observation.second;

    // if the new one is seen in obs_shot, there was a mismatch
    // Thus, erase the observation of the old_lm
    if (new_lm->IsObservedInShot(obs_shot)) {
      obs_shot->RemoveLandmarkObservation(obs_feat_id);
    } else {
      // replace, should be the same as AddObservation
      obs_shot->AddLandmarkObservation(new_lm, obs_feat_id);
      new_lm->AddObservation(obs_shot, obs_feat_id);
    }
  }
  // TODO: This basically takes all the observations from the old lm
  // Similar to OpenVSLAM but might not be completely correct
  // because the old and the replaced landmark might have coinciding
  // observations
  new_lm->slam_data_.IncreaseNumObserved(old_lm->slam_data_.GetNumObserved());
  new_lm->slam_data_.IncreaseNumObservable(
      old_lm->slam_data_.GetNumObservable());
  // 3) Remove from landmarks
  landmarks_.erase(old_lm->id_);
}

// TODO: Removing the camera might be problematic when it is still in use
// void
// Map::RemoveCamera(const CameraId cam_id)
// {
//   const auto& cam_it = cameras_.find(cam_id);
//   if (cam_it != cameras_.end())
//   {
//     cameras_.erase(cam_it);
//   }
// }

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
                  cam.GetParameterValue(Camera::Parameters::Cy)), distortion);
      }
      case ProjectionType::FISHEYE:
        return Camera::CreateFisheyeCamera(
            cam.GetParameterValue(Camera::Parameters::Focal),
            cam.GetParameterValue(Camera::Parameters::K1),
            cam.GetParameterValue(Camera::Parameters::K2));
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

std::vector<Camera*> Map::GetCameras() {
  std::vector<Camera*> cameras;
  cameras.reserve(cameras_.size());
  for (auto& cam_mod : cameras_) {
    cameras.push_back(&cam_mod.second);
  }
  return cameras;
}

Camera* Map::GetCamera(const CameraId& cam_id) 
{ 
  auto it = cameras_.find(cam_id);
  if (it == cameras_.end())
  {
    return nullptr;
  }
  return &it->second; 
}

};  // namespace map
