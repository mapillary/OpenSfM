#pragma once

#include <geo/geo.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
#include <geometry/similarity.h>
#include <map/dataviews.h>
#include <map/defines.h>
#include <map/landmark.h>
#include <map/rig.h>
#include <map/shot.h>
#include <map/tracks_manager.h>

#include <Eigen/Core>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
namespace map {

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Camera Methods
  geometry::Camera& GetCamera(const CameraId& cam_id);
  const geometry::Camera& GetCamera(const CameraId& cam_id) const;
  geometry::Camera& CreateCamera(const geometry::Camera& cam);
  CameraView GetCameraView() { return CameraView(*this); }
  bool HasCamera(const CameraId& cam_id) const {
    return cameras_.count(cam_id) > 0;
  }
  const std::unordered_map<CameraId, geometry::Camera>& GetCameras() const {
    return cameras_;
  }
  std::unordered_map<CameraId, geometry::Camera>& GetCameras() {
    return cameras_;
  }

  // Shot Methods

  // Creation
  Shot& CreateShot(const ShotId& shot_id, const CameraId& camera_id);
  Shot& CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                   const geometry::Pose& pose);
  Shot& CreateShot(const ShotId& shot_id, const geometry::Camera* const cam,
                   const geometry::Pose& pose = geometry::Pose());

  // Getters
  const Shot& GetShot(const ShotId& shot_id) const;
  Shot& GetShot(const ShotId& shot_id);
  bool HasShot(const ShotId& shot_id) const {
    return shots_.find(shot_id) != shots_.end();
  }
  const std::unordered_map<ShotId, Shot>& GetShots() const { return shots_; }
  std::unordered_map<ShotId, Shot>& GetShots() { return shots_; }

  // Update
  Shot& UpdateShot(const Shot& other_shot);
  void RemoveShot(const ShotId& shot_id);
  ShotView GetShotView() { return ShotView(*this); }

  // PanoShots

  // Creation
  Shot& CreatePanoShot(const ShotId& shot_id, const CameraId&,
                       const geometry::Pose& pose);
  Shot& CreatePanoShot(const ShotId& shot_id, const CameraId&);
  Shot& CreatePanoShot(const ShotId& shot_id, const geometry::Camera* const cam,
                       const geometry::Pose& pose);

  // Getters
  Shot& GetPanoShot(const ShotId& shot_id);
  const Shot& GetPanoShot(const ShotId& shot_id) const;
  bool HasPanoShot(const ShotId& shot_id) const {
    return pano_shots_.find(shot_id) != pano_shots_.end();
  }
  const std::unordered_map<ShotId, Shot>& GetPanoShots() const {
    return pano_shots_;
  }
  std::unordered_map<ShotId, Shot>& GetPanoShots() { return pano_shots_; }

  // Update
  void RemovePanoShot(const ShotId& shot_id);
  Shot& UpdatePanoShot(const Shot& other_shot);
  PanoShotView GetPanoShotView() { return PanoShotView(*this); }

  // Rigs

  // Creation
  RigCamera& CreateRigCamera(const map::RigCamera& rig_camera);
  RigInstance& CreateRigInstance(
      const map::RigInstanceId& instance_id,
      const std::map<map::ShotId, map::RigCameraId>& instance_shots);

  // Update
  RigInstance& UpdateRigInstance(const RigInstance& other_rig_instance);

  // Getters
  size_t NumberOfRigCameras() const;
  RigCamera& GetRigCamera(const RigCameraId& rig_camera_id);
  const std::unordered_map<RigCameraId, RigCamera>& GetRigCameras() const {
    return rig_cameras_;
  }
  std::unordered_map<RigCameraId, RigCamera>& GetRigCameras() {
    return rig_cameras_;
  }
  bool HasRigCamera(const RigCameraId& rig_camera_id) const;

  size_t NumberOfRigInstances() const;
  RigInstance& GetRigInstance(const RigInstanceId& instance_id);
  const RigInstance& GetRigInstance(const RigInstanceId& instance_id) const;
  const std::unordered_map<RigInstanceId, RigInstance>& GetRigInstances()
      const {
    return rig_instances_;
  }
  std::unordered_map<RigInstanceId, RigInstance>& GetRigInstances() {
    return rig_instances_;
  }

  bool HasRigInstance(const RigInstanceId& instance_id) const;

  // Landmark

  // Creation
  Landmark& CreateLandmark(const LandmarkId& lm_id, const Vec3d& global_pos);

  // Getters
  const Landmark& GetLandmark(const LandmarkId& lm_id) const;
  Landmark& GetLandmark(const LandmarkId& lm_id);
  const std::unordered_map<LandmarkId, Landmark>& GetLandmarks() const {
    return landmarks_;
  }
  std::unordered_map<LandmarkId, Landmark>& GetLandmarks() {
    return landmarks_;
  }
  bool HasLandmark(const LandmarkId& lm_id) const {
    return landmarks_.count(lm_id) > 0;
  }
  LandmarkView GetLandmarkView() { return LandmarkView(*this); }

  // Update
  void RemoveLandmark(const Landmark* const lm);
  void RemoveLandmark(const LandmarkId& lm_id);

  // Observation
  void AddObservation(Shot* const shot, Landmark* const lm,
                      const Observation& obs);
  void AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                      const Observation& obs);
  void RemoveObservation(const ShotId& shot_id, const LandmarkId& lm_id);
  void ClearObservationsAndLandmarks();

  // Map information and access methods
  size_t NumberOfShots() const { return shots_.size(); }
  size_t NumberOfPanoShots() const { return pano_shots_.size(); }
  size_t NumberOfLandmarks() const { return landmarks_.size(); }
  size_t NumberOfCameras() const { return cameras_.size(); }
  size_t NumberOfBiases() const { return bias_.size(); }

  // Bias
  BiasView GetBiasView() { return BiasView(*this); }
  geometry::Similarity& GetBias(const CameraId& camera_id);
  void SetBias(const CameraId& camera_id,
               const geometry::Similarity& transform);
  bool HasBias(const CameraId& cam_id) const {
    return bias_.find(cam_id) != bias_.end();
  }
  const std::unordered_map<CameraId, geometry::Similarity>& GetBiases() const {
    return bias_;
  }
  std::unordered_map<CameraId, geometry::Similarity>& GetBiases() {
    return bias_;
  }

  // TopocentricConverter
  const geo::TopocentricConverter& GetTopocentricConverter() const {
    return topo_conv_;
  }
  void SetTopocentricConverter(const double lat, const double longitude,
                               const double alt) {
    topo_conv_.lat_ = lat;
    topo_conv_.long_ = longitude;
    topo_conv_.alt_ = alt;
  }
  TracksManager ToTracksManager() const;

  // Tracks manager x Reconstruction intersection functions
  enum ErrorType { Pixel = 0x0, Normalized = 0x1, Angular = 0x2 };
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Vec2d> >
  ComputeReprojectionErrors(const TracksManager& tracks_manager,
                            const ErrorType& error_type) const;
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Observation> >
  GetValidObservations(const TracksManager& tracks_manager) const;

 private:
  std::unordered_map<CameraId, geometry::Camera> cameras_;
  std::unordered_map<CameraId, geometry::Similarity> bias_;
  std::unordered_map<ShotId, Shot> shots_;
  std::unordered_map<ShotId, Shot> pano_shots_;
  std::unordered_map<LandmarkId, Landmark> landmarks_;
  std::unordered_map<RigInstanceId, RigInstance> rig_instances_;
  std::unordered_map<RigCameraId, RigCamera> rig_cameras_;

  geo::TopocentricConverter topo_conv_;
};

}  // namespace map
