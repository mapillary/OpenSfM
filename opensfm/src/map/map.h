#pragma once

#include <Eigen/Core>
#include <map>
#include <memory>
#include <unordered_map>

#include <map/defines.h>

#include <geo/geo.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
#include <map/dataviews.h>
#include <map/defines.h>
#include <sfm/tracks_manager.h>
namespace map {
class Shot;
class Landmark;

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Camera Methods
  Camera* GetCamera(const CameraId& cam_id);
  Camera* CreateCamera(const Camera& cam);
  CameraView GetCameraView() { return CameraView(*this); }
  bool HasCamera(const CameraId& cam_id) const {
    return cameras_.count(cam_id) > 0;
  }
  const std::unordered_map<CameraId, Camera>& GetCameras() const {
    return cameras_;
  }
  std::unordered_map<CameraId, Camera>& GetCameras() { return cameras_; }

  // Shot Methods
  Shot* CreateShot(const ShotId& shot_id, const CameraId& camera_id);
  Shot* CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                   const geometry::Pose& pose);
  Shot* CreateShot(const ShotId& shot_id, const Camera* const cam,
                   const geometry::Pose& pose = geometry::Pose());
  Shot* GetShot(const ShotId& shot_id);
  Shot* UpdateShot(const Shot& other_shot);
  void RemoveShot(const ShotId& shot_id);
  bool HasLandmark(const LandmarkId& lm_id) const {
    return landmarks_.count(lm_id) > 0;
  }
  bool HasShot(const ShotId& shot_id) const {
    return shots_.find(shot_id) != shots_.end();
  }
  const std::unordered_map<ShotId, Shot>& GetShots() const { return shots_; }
  std::unordered_map<ShotId, Shot>& GetShots() { return shots_; }

  ShotView GetShotView() { return ShotView(*this); }

  // PanoShots
  Shot* CreatePanoShot(const ShotId& shot_id, const CameraId&,
                       const geometry::Pose& pose);
  Shot* CreatePanoShot(const ShotId& shot_id, const CameraId&);
  Shot* CreatePanoShot(const ShotId& shot_id, const Camera* const cam,
                       const geometry::Pose& pose);
  Shot* UpdatePanoShot(const Shot& other_shot);
  Shot* GetPanoShot(const ShotId& shot_id);
  void RemovePanoShot(const ShotId& shot_id);
  bool HasPanoShot(const ShotId& shot_id) const {
    return pano_shots_.find(shot_id) != pano_shots_.end();
  }
  const std::unordered_map<ShotId, Shot>& GetPanoShots() const {
    return pano_shots_;
  }
  std::unordered_map<ShotId, Shot>& GetPanoShots() { return pano_shots_; }
  PanoShotView GetPanoShotView() { return PanoShotView(*this); }

  // Landmark
  Landmark* CreateLandmark(const LandmarkId& lm_id, const Vec3d& global_pos);
  Landmark* GetLandmark(const LandmarkId& lm_id);
  void RemoveLandmark(const Landmark* const lm);
  void RemoveLandmark(const LandmarkId& lm_id);
  void ReplaceLandmark(Landmark* old_lm, Landmark* new_lm);
  const std::unordered_map<LandmarkId, Landmark>& GetLandmarks() const {
    return landmarks_;
  }
  std::unordered_map<LandmarkId, Landmark>& GetLandmarks() {
    return landmarks_;
  }
  LandmarkView GetLandmarkView() { return LandmarkView(*this); }

  // Observation methods
  void AddObservation(Shot* const shot, Landmark* const lm,
                      const FeatureId feat_id);
  void AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                      const FeatureId feat_id);
  void AddObservation(Shot* const shot, Landmark* const lm,
                      const Observation& obs);
  void AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                      const Observation& obs);
  void RemoveObservation(Shot* const shot, Landmark* const lm,
                         const FeatureId feat_id);
  void RemoveObservation(const ShotId& shot_id, const LandmarkId& lm_id);
  void ClearObservationsAndLandmarks();

  // Map information and access methods
  size_t NumberOfShots() const { return shots_.size(); }
  size_t NumberOfPanoShots() const { return pano_shots_.size(); }
  size_t NumberOfLandmarks() const { return landmarks_.size(); }
  size_t NumberOfCameras() const { return cameras_.size(); }
  std::map<Landmark*, FeatureId> GetObservationsOfShot(const Shot* shot);
  std::map<Shot*, FeatureId> GetObservationsOfPoint(const Landmark* point);

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

 private:
  std::unordered_map<CameraId, Camera> cameras_;
  std::unordered_map<ShotId, Shot> shots_;
  std::unordered_map<ShotId, Shot> pano_shots_;
  std::unordered_map<LandmarkId, Landmark> landmarks_;

  geo::TopocentricConverter topo_conv_;

  LandmarkUniqueId landmark_unique_id_ = 0;
  ShotUniqueId shot_unique_id_ = 0;
  ShotUniqueId pano_shot_unique_id_ = 0;
};

}  // namespace map
