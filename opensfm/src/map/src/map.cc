#include <geometry/pose.h>
#include <map/defines.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/rig.h>
#include <map/shot.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_set>

namespace {
void AssignShot(map::Shot& to, const map::Shot& from) {
  to.merge_cc = from.merge_cc;
  to.scale = from.scale;
  to.SetShotMeasurements(from.GetShotMeasurements());
  to.SetCovariance(from.GetCovariance());
}
}  // namespace
namespace map {

std::unique_ptr<Map> Map::DeepCopy(const Map& map, bool copy_observations) {
  auto map_copy = std::make_unique<Map>();
  map_copy->topo_conv_ = map.topo_conv_;

  for (const auto& camera : map.GetCameras()) {
    map_copy->CreateCamera(camera.second);
  }

  const auto& shots = map.GetShots();
  for (const auto& shot : shots) {
    if (map_copy->HasShot(shot.first)) {
      continue;
    }
    map_copy->UpdateShotWithRig(shot.second, false);
  }

  const auto& pano_shots = map.GetPanoShots();
  for (const auto& pano_shot : pano_shots) {
    if (map_copy->HasPanoShot(pano_shot.first)) {
      continue;
    }
    map_copy->UpdateShotWithRig(pano_shot.second, true);
  }

  for (const auto& landmark : map.GetLandmarks()) {
    map_copy->CreateLandmark(landmark.first, landmark.second.GetGlobalPos());
  }

  if (copy_observations) {
    for (const auto& shot : shots) {
      for (const auto& landmark_n_obs : shot.second.GetLandmarkObservations()) {
        map_copy->AddObservation(shot.first, landmark_n_obs.first->id_,
                                 landmark_n_obs.second);
      }
    }
  }

  for (const auto& bias : map.GetBiases()) {
    map_copy->SetBias(bias.first, bias.second);
  }

  return map_copy;
}

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

void Map::CleanLandmarksBelowMinObservations(const size_t min_observations) {
  for (auto it = landmarks_.begin(); it != landmarks_.end();) {
    const auto& landmark = it->second;
    if (landmark.NumberOfObservations() < min_observations) {
      // 2) Remove all its observation
      const auto& observations = landmark.GetObservations();
      for (const auto& obs : observations) {
        Shot* shot = obs.first;
        const auto feat_id = obs.second;
        shot->RemoveLandmarkObservation(feat_id);
      }
      // 3) Remove from landmarks
      it = landmarks_.erase(it);
    } else {
      ++it;
    }
  }
}

/**
 * Creates a shot and returns a reference to it
 *
 * @param shot_id       unique id of the shot
 * @param camera_id     unique id of EXISTING camera
 * @param rig_camera_id unique id of EXISTING rig camera
 * @param instance_id   unique id of EXISTING rig instance
 * @param pose          position in the 3D world
 *
 * @returns             returns reference to created or existing shot
 */
Shot& Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                      const RigCameraId& rig_camera_id,
                      const RigInstanceId& instance_id,
                      const geometry::Pose& pose) {
  auto it_exist = shots_.find(shot_id);
  if (it_exist == shots_.end())  // create
  {
    const auto& camera = GetCamera(camera_id);
    auto& rig_instance = GetRigInstance(instance_id);
    auto& rig_camera = GetRigCamera(rig_camera_id);
    auto it =
        shots_.emplace(std::piecewise_construct, std::forward_as_tuple(shot_id),
                       std::forward_as_tuple(shot_id, &camera, &rig_instance,
                                             &rig_camera, pose));
    return it.first->second;
  } else {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
}
Shot& Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                      const RigCameraId& rig_camera_id,
                      const RigInstanceId& instance_id) {
  auto it_exist = shots_.find(shot_id);
  if (it_exist == shots_.end())  // create
  {
    const auto& camera = GetCamera(camera_id);
    auto& rig_instance = GetRigInstance(instance_id);
    auto& rig_camera = GetRigCamera(rig_camera_id);
    auto it = shots_.emplace(
        std::piecewise_construct, std::forward_as_tuple(shot_id),
        std::forward_as_tuple(shot_id, &camera, &rig_instance, &rig_camera));
    return it.first->second;
  } else {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
}

void Map::RemoveShot(const ShotId& shot_id) {
  // 1) Find the point
  const auto& shot_it = shots_.find(shot_id);
  if (shot_it != shots_.end()) {
    auto& shot = shot_it->second;
    shot.GetRigInstance()->RemoveShot(shot_id);
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

/**
 * Creates a pano shot and returns a reference to it
 *
 * @param shot_id       unique id of the shot
 * @param camera_id     unique id of EXISTING camera
 * @param rig_camera_id unique id of EXISTING rig camera
 * @param instance_id   unique id of EXISTING rig instance
 * @param pose          position in the 3D world
 * @returns             returns reference to created or existing shot
 */
Shot& Map::CreatePanoShot(const ShotId& shot_id, const CameraId& camera_id,
                          const RigCameraId& rig_camera_id,
                          const RigInstanceId& instance_id,
                          const geometry::Pose& pose) {
  auto it_exist = pano_shots_.find(shot_id);
  if (it_exist == pano_shots_.end()) {
    const auto& camera = GetCamera(camera_id);
    auto& rig_instance = GetRigInstance(instance_id);
    auto& rig_camera = GetRigCamera(rig_camera_id);
    auto it = pano_shots_.emplace(
        std::piecewise_construct, std::forward_as_tuple(shot_id),
        std::forward_as_tuple(shot_id, &camera, &rig_instance, &rig_camera,
                              pose));
    return it.first->second;
  } else {
    throw std::runtime_error("PanoShot " + shot_id + " already exists.");
  }
}

void Map::RemovePanoShot(const ShotId& shot_id) {
  const auto& shot_it = pano_shots_.find(shot_id);
  if (shot_it != pano_shots_.end()) {
    const auto& shot = shot_it->second;
    shot.GetRigInstance()->RemoveShot(shot_id);
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
  auto it = cameras_.emplace(cam.id, cam);
  bias_.emplace(cam.id, geometry::Similarity());
  return it.first->second;
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

void Map::UpdateShotWithRig(const Shot& other_shot, bool is_panoshot) {
  const auto rig_instance = other_shot.GetRigInstance();
  const auto& instance_id = rig_instance->id;
  const bool has_instance = HasRigInstance(instance_id);
  if (!has_instance) {
    CreateRigInstance(instance_id);
  }
  for (const auto& instance_shot : rig_instance->GetShots()) {
    const auto& shot_id = instance_shot.first;
    const auto& shot = instance_shot.second;

    const auto camera = shot->GetCamera();
    const auto& camera_id = camera->id;
    if (!HasCamera(camera_id)) {
      CreateCamera(*camera);
    }

    const auto rig_camera = shot->GetRigCamera();
    const auto& rig_camera_id = rig_camera->id;
    if (!HasRigCamera(rig_camera_id)) {
      CreateRigCamera(*rig_camera);
    }

    const bool has_shot = is_panoshot ? HasPanoShot(shot_id) : HasShot(shot_id);
    if (!has_shot) {
      Shot* new_shot = nullptr;
      if (is_panoshot) {
        new_shot = &CreatePanoShot(shot_id, camera_id, rig_camera_id,
                                   instance_id, *shot->GetPose());
      } else {
        new_shot = &CreateShot(shot_id, camera_id, rig_camera_id, instance_id,
                               *shot->GetPose());
      }
      AssignShot(*new_shot, *shot);
    }
  }
  GetRigInstance(instance_id)
      .UpdateInstancePoseWithShot(other_shot.id_, *other_shot.GetPose());
}

Shot& Map::UpdateShot(const Shot& other_shot) {
  auto it_exist = shots_.find(other_shot.id_);
  if (it_exist == shots_.end()) {
    throw std::runtime_error("Shot " + other_shot.id_ + " does not exists.");
  } else {
    auto& shot = it_exist->second;
    UpdateShotWithRig(other_shot, false);
    AssignShot(shot, other_shot);
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
    UpdateShotWithRig(other_shot, true);
    AssignShot(shot, other_shot);

    return shot;
  }
}

RigCamera& Map::CreateRigCamera(const map::RigCamera& rig_camera) {
  auto it_exist = rig_cameras_.find(rig_camera.id);
  if (it_exist != rig_cameras_.end()) {
    throw std::runtime_error("RigCamera " + rig_camera.id + " already exists.");
  }

  auto it = rig_cameras_.emplace(rig_camera.id, rig_camera);
  return it.first->second;
}

RigInstance& Map::CreateRigInstance(const map::RigInstanceId& instance_id) {
  auto it_exist = rig_instances_.find(instance_id);
  if (it_exist != rig_instances_.end()) {
    throw std::runtime_error("RigInstance " + instance_id + " already exists.");
  }

  // Create instance and add its shots
  auto it = rig_instances_.emplace(std::piecewise_construct,
                                   std::forward_as_tuple(instance_id),
                                   std::forward_as_tuple(instance_id));
  return it.first->second;
}

void Map::RemoveRigInstance(const map::RigInstanceId& instance_id) {
  auto it_exist = rig_instances_.find(instance_id);
  if (it_exist == rig_instances_.end()) {
    throw std::runtime_error("Rig instance does not exists.");
  } else {
    rig_instances_.erase(instance_id);
  }
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

size_t Map::NumberOfRigCameras() const { return rig_cameras_.size(); }

RigCamera& Map::GetRigCamera(const RigCameraId& rig_camera_id) {
  const auto& it = rig_cameras_.find(rig_camera_id);
  if (it == rig_cameras_.end()) {
    throw std::runtime_error("Accessing invalid RigCameraID " + rig_camera_id);
  }
  return it->second;
}

bool Map::HasRigCamera(const RigCameraId& rig_camera_id) const {
  return rig_cameras_.find(rig_camera_id) != rig_cameras_.end();
}

size_t Map::NumberOfRigInstances() const { return rig_instances_.size(); }

RigInstance& Map::GetRigInstance(const RigInstanceId& instance_id) {
  const auto& it = rig_instances_.find(instance_id);
  if (it == rig_instances_.end()) {
    throw std::runtime_error("Accessing invalid RigInstance index");
  }
  return it->second;
}

const RigInstance& Map::GetRigInstance(const RigInstanceId& instance_id) const {
  const auto& it = rig_instances_.find(instance_id);
  if (it == rig_instances_.end()) {
    throw std::runtime_error("Accessing invalid RigInstance index");
  }
  return it->second;
}

bool Map::HasRigInstance(const RigInstanceId& instance_id) const {
  return rig_instances_.find(instance_id) != rig_instances_.end();
}

geometry::Similarity& Map::GetBias(const CameraId& camera_id) {
  const auto it = bias_.find(camera_id);
  if (it == bias_.end()) {
    throw std::runtime_error("Accessing invalid CameraID " + camera_id);
  }
  return it->second;
}

void Map::SetBias(const CameraId& camera_id,
                  const geometry::Similarity& transform) {
  auto it = bias_.find(camera_id);
  if (it == bias_.end()) {
    throw std::runtime_error("Accessing invalid CameraID " + camera_id);
  }
  it->second = transform;
}

std::unordered_map<ShotId, std::unordered_map<LandmarkId, Vec2d> >
Map::ComputeReprojectionErrors(const TracksManager& tracks_manager,
                               const Map::ErrorType& error_type) const {
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

      if (error_type == Map::ErrorType::Pixel) {
        const Vec2d error_2d =
            (track_n_obs.second.point -
             shot.Project(find_landmark->second.GetGlobalPos()));
        per_shot[track_n_obs.first] = error_2d;
      }
      if (error_type == Map::ErrorType::Normalized) {
        const Vec2d error_2d =
            (track_n_obs.second.point -
             shot.Project(find_landmark->second.GetGlobalPos()));
        per_shot[track_n_obs.first] = error_2d / track_n_obs.second.scale;
      }
      if (error_type == Map::ErrorType::Angular) {
        const Vec3d point =
            (find_landmark->second.GetGlobalPos() - shot.GetPose()->GetOrigin())
                .normalized();
        const Vec3d bearing =
            shot.Bearing(track_n_obs.second.point).normalized();
        const double angle = std::acos(point.dot(bearing));
        per_shot[track_n_obs.first] = Vec2d::Constant(angle);
      }
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
