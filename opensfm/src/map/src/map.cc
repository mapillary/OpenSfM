#include <geometry/pose.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/rig.h>
#include <map/shot.h>

#include <unordered_set>

namespace map {

void Map::AddObservation(Shot* const shot, Landmark* const lm,
                         const Observation& obs) {
  lm->AddObservation(shot, obs.feature_id);
  shot->CreateObservation(lm, obs);
}

void Map::AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                         const Observation& obs) {
  auto& shot = GetShot(shot_id);
  auto& lm = GetLandmark(lm_id);
  AddObservation(&shot, &lm, obs);
}

void Map::RemoveObservation(const ShotId& shot_id, const LandmarkId& lm_id) {
  auto& shot = GetShot(shot_id);
  auto& lm = GetLandmark(lm_id);
  shot.RemoveLandmarkObservation(lm.GetObservationIdInShot(&shot));
  lm.RemoveObservation(&shot);
}

const Shot& Map::GetShot(const ShotId& shot_id) const {
  const auto& it = shots_.find(shot_id);
  if (it == shots_.end()) {
    throw std::runtime_error("Accessing invalid ShotID " + shot_id);
  }
  return it->second;
}
Shot& Map::GetShot(const ShotId& shot_id) {
  const auto& it = shots_.find(shot_id);
  if (it == shots_.end()) {
    throw std::runtime_error("Accessing invalid ShotID " + shot_id);
  }
  return it->second;
}

Shot& Map::GetPanoShot(const ShotId& shot_id) {
  const auto& it = pano_shots_.find(shot_id);
  if (it == pano_shots_.end()) {
    throw std::runtime_error("Accessing invalid PanoShotID " + shot_id);
  }
  return it->second;
}

const Shot& Map::GetPanoShot(const ShotId& shot_id) const {
  const auto& it = pano_shots_.find(shot_id);
  if (it == pano_shots_.end()) {
    throw std::runtime_error("Accessing invalid PanoShotID " + shot_id);
  }
  return it->second;
}

const Landmark& Map::GetLandmark(const LandmarkId& lm_id) const {
  const auto& it = landmarks_.find(lm_id);
  if (it == landmarks_.end()) {
    throw std::runtime_error("Accessing invalid LandmarkId " + lm_id);
  }
  return it->second;
}
Landmark& Map::GetLandmark(const LandmarkId& lm_id) {
  const auto& it = landmarks_.find(lm_id);
  if (it == landmarks_.end()) {
    throw std::runtime_error("Accessing invalid LandmarkId " + lm_id);
  }
  return it->second;
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

Shot& Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id) {
  return CreateShot(shot_id, camera_id, geometry::Pose());
}

/**
 * Creates a shot and returns a reference to it
 *
 * @param shot_id       unique id of the shot
 * @param camera        previously created camera
 * @param global_pos    position in the 3D world
 *
 * @returns             returns reference to created or existing shot
 */
Shot& Map::CreateShot(const ShotId& shot_id, const geometry::Camera* const cam,
                      const geometry::Pose& pose) {
  auto it_exist = shots_.find(shot_id);
  if (it_exist == shots_.end())  // create
  {
    auto it =
        shots_.emplace(std::piecewise_construct, std::forward_as_tuple(shot_id),
                       std::forward_as_tuple(shot_id, cam, pose));

    it.first->second.unique_id_ = shot_unique_id_;
    shot_unique_id_++;
    return it.first->second;
  } else {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
}

/**
 * Creates a shot and returns a reference to it
 *
 * @param shot_id       unique id of the shot
 * @param camera_id     unique id of EXISTING camera
 * @param global_pos    position in the 3D world
 *
 * @returns             returns reference to created or existing shot
 */
Shot& Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                      const geometry::Pose& pose) {
  return CreateShot(shot_id, &GetCamera(camera_id), pose);
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

Shot& Map::CreatePanoShot(const ShotId& shot_id, const CameraId& camera_id) {
  return CreatePanoShot(shot_id, camera_id, geometry::Pose());
}

/**
 * Creates a pano shot and returns a reference to it
 *
 * @param shot_id       unique id of the shot
 * @param camera        previously created camera
 * @param global_pos    position in the 3D world
 *
 * @returns             returns reference to created or existing shot
 */
Shot& Map::CreatePanoShot(const ShotId& shot_id,
                          const geometry::Camera* const cam,
                          const geometry::Pose& pose) {
  auto it_exist = pano_shots_.find(shot_id);
  if (it_exist == pano_shots_.end()) {
    auto it = pano_shots_.emplace(std::piecewise_construct,
                                  std::forward_as_tuple(shot_id),
                                  std::forward_as_tuple(shot_id, cam, pose));
    it.first->second.unique_id_ = pano_shot_unique_id_;
    pano_shot_unique_id_++;
    return it.first->second;
  } else {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
}

Shot& Map::CreatePanoShot(const ShotId& shot_id, const CameraId& camera_id,
                          const geometry::Pose& pose) {
  return CreatePanoShot(shot_id, &GetCamera(camera_id), pose);
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
 * Creates a landmark and returns a reference to it
 *
 * @param lm_Id       unique id of the landmark
 * @param global_pos  3D position of the landmark
 * @param name        name of the landmark
 *
 * @returns           reference to the created or already existing lm
 */
Landmark& Map::CreateLandmark(const LandmarkId& lm_id,
                              const Vec3d& global_pos) {
  auto it_exist = landmarks_.find(lm_id);
  if (it_exist == landmarks_.end()) {
    auto it = landmarks_.emplace(std::piecewise_construct,
                                 std::forward_as_tuple(lm_id),
                                 std::forward_as_tuple(lm_id, global_pos));
    it.first->second.unique_id_ = landmark_unique_id_;
    landmark_unique_id_++;
    return it.first->second;
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

geometry::Camera& Map::CreateCamera(const geometry::Camera& cam) {
  auto it = cameras_.emplace(std::make_pair(cam.id, cam));
  return it.first->second;
  ;
}

geometry::Camera& Map::GetCamera(const CameraId& cam_id) {
  auto it = cameras_.find(cam_id);
  if (it == cameras_.end()) {
    throw std::runtime_error("Accessing invalid CameraId " + cam_id);
  }
  return it->second;
}

const geometry::Camera& Map::GetCamera(const CameraId& cam_id) const {
  auto it = cameras_.find(cam_id);
  if (it == cameras_.end()) {
    throw std::runtime_error("Accessing invalid CameraId " + cam_id);
  }
  return it->second;
}

Shot& Map::UpdateShot(const Shot& other_shot) {
  auto it_exist = shots_.find(other_shot.id_);
  if (it_exist == shots_.end()) {
    throw std::runtime_error("Shot " + other_shot.id_ + " does not exists.");
  } else {
    auto& shot = it_exist->second;
    shot.merge_cc = other_shot.merge_cc;
    shot.scale = other_shot.scale;
    shot.SetShotMeasurements(other_shot.GetShotMeasurements());
    shot.SetCovariance(other_shot.GetCovariance());
    shot.SetPose(*other_shot.GetPose());
    return shot;
  }
}

Shot& Map::UpdatePanoShot(const Shot& other_shot) {
  auto it_exist = pano_shots_.find(other_shot.id_);
  if (it_exist == pano_shots_.end()) {
    throw std::runtime_error("Pano shot " + other_shot.id_ +
                             " does not exists.");
  } else {
    auto& shot = it_exist->second;
    shot.merge_cc = other_shot.merge_cc;
    shot.scale = other_shot.scale;
    shot.SetShotMeasurements(other_shot.GetShotMeasurements());
    shot.SetCovariance(other_shot.GetCovariance());
    shot.SetPose(*other_shot.GetPose());
    return shot;
  }
}

RigModel& Map::CreateRigModel(const map::RigModel& model) {
  auto it = rig_models_.emplace(std::make_pair(model.id, model));
  return it.first->second;
}

RigInstance& Map::CreateRigInstance(
    map::RigModel* rig_model, const map::RigInstanceId& instance_id,
    const std::map<map::ShotId, map::RigCameraId>& instance_shots) {
  auto it_rig_model_exist = rig_models_.find(rig_model->id);
  if (it_rig_model_exist == rig_models_.end()) {
    throw std::runtime_error("Rig model" + rig_model->id + " does not exists.");
  }

  // Create instance and add its shots
  auto it = rig_instances_.emplace(
      std::piecewise_construct, std::forward_as_tuple(instance_id),
      std::forward_as_tuple(rig_model, instance_id));
  auto& instance = it.first->second;
  for (const auto& shot_id : instance_shots) {
    auto it_shot_exist = shots_.find(shot_id.first);
    if (it_shot_exist == shots_.end()) {
      throw std::runtime_error("Instance shot " + shot_id.first +
                               " does not exists.");
    }
    instance.AddShot(shot_id.second, &it_shot_exist->second);
  }
  return instance;
}

RigInstance& Map::UpdateRigInstance(const RigInstance& other_rig_instance) {
  auto it_exist = rig_instances_.find(other_rig_instance.id);
  if (it_exist == rig_instances_.end()) {
    throw std::runtime_error("Rig instance does not exists.");
  } else {
    auto& rig_instance = it_exist->second;
    rig_instance = other_rig_instance;
    return rig_instance;
  }
}

size_t Map::NumberOfRigModels() const { return rig_models_.size(); }

RigModel& Map::GetRigModel(const RigModelId& rig_model_id) {
  const auto& it = rig_models_.find(rig_model_id);
  if (it == rig_models_.end()) {
    throw std::runtime_error("Accessing invalid RigModelID " + rig_model_id);
  }
  return it->second;
}

bool Map::HasRigModel(const RigModelId& rig_model_id) const {
  return rig_models_.find(rig_model_id) != rig_models_.end();
}

size_t Map::NumberOfRigInstances() const { return rig_instances_.size(); }

RigInstance& Map::GetRigInstance(const RigInstanceId& instance_id) {
  const auto& it = rig_instances_.find(instance_id);
  if (it == rig_instances_.end()) {
    throw std::runtime_error("Accessing invalid RigInstance index");
  }
  return it->second;
}

bool Map::HasRigInstance(const RigInstanceId& instance_id) const {
  return rig_instances_.find(instance_id) != rig_instances_.end();
}

std::unordered_map<ShotId, std::unordered_map<LandmarkId, Vec2d> >
Map::ComputeReprojectionErrors(const TracksManager& tracks_manager,
                               bool scaled) const {
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Vec2d> > errors;
  for (const auto& shot_id : tracks_manager.GetShotIds()) {
    const auto find_shot = shots_.find(shot_id);
    if (find_shot == shots_.end()) {
      continue;
    }
    const auto& shot = find_shot->second;
    auto& per_shot = errors[shot_id];
    for (const auto& track_n_obs :
         tracks_manager.GetShotObservations(shot_id)) {
      const auto find_landmark = landmarks_.find(track_n_obs.first);
      if (find_landmark == landmarks_.end()) {
        continue;
      }

      const Vec2d error_2d =
          (track_n_obs.second.point -
           shot.Project(find_landmark->second.GetGlobalPos()));
      const auto scale = scaled ? track_n_obs.second.scale : 1.0;
      per_shot[track_n_obs.first] = error_2d / scale;
    }
  }
  return errors;
}

std::unordered_map<ShotId, std::unordered_map<LandmarkId, Observation> >
Map::GetValidObservations(const TracksManager& tracks_manager) const {
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Observation> >
      observations;
  for (const auto& shot_id : tracks_manager.GetShotIds()) {
    const auto find_shot = shots_.find(shot_id);
    if (find_shot == shots_.end()) {
      continue;
    }
    auto& per_shot = observations[shot_id];
    for (const auto& track_n_obs :
         tracks_manager.GetShotObservations(shot_id)) {
      const auto find_landmark = landmarks_.find(track_n_obs.first);
      if (find_landmark == landmarks_.end()) {
        continue;
      }
      per_shot[track_n_obs.first] = track_n_obs.second;
    }
  }
  return observations;
}

TracksManager Map::ToTracksManager() const {
  TracksManager manager;
  for (const auto& shot_pair : shots_) {
    for (const auto& lm_obs : shot_pair.second.GetLandmarkObservations()) {
      manager.AddObservation(shot_pair.first, lm_obs.first->id_, lm_obs.second);
    }
  }
  for (const auto& shot_pair : pano_shots_) {
    for (const auto& lm_obs : shot_pair.second.GetLandmarkObservations()) {
      manager.AddObservation(shot_pair.first, lm_obs.first->id_, lm_obs.second);
    }
  }
  return manager;
}

};  // namespace map
