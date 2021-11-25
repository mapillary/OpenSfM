#pragma once
#include <geometry/camera.h>
#include <geometry/similarity.h>
#include <map/landmark.h>
#include <map/rig.h>
#include <map/shot.h>

#include <deque>
#include <unordered_map>

namespace map {
class Map;
class ShotView {
 public:
  ShotView(Map& map);
  Shot& GetShot(const map::ShotId& shot_id);
  bool HasShot(const map::ShotId& shot_id) const;
  const std::unordered_map<ShotId, Shot>& GetShots() const;
  size_t NumberOfShots() const;

 private:
  Map& map_;
};

class PanoShotView {
 public:
  PanoShotView(Map& map);
  Shot& GetShot(const map::ShotId& shot_id);
  bool HasShot(const map::ShotId& shot_id) const;
  const std::unordered_map<ShotId, Shot>& GetShots() const;
  size_t NumberOfShots() const;

 private:
  Map& map_;
};

class LandmarkView {
 public:
  LandmarkView(Map& map);
  Landmark& GetLandmark(const LandmarkId& lm_id);
  bool HasLandmark(const LandmarkId& lm_id) const;
  const std::unordered_map<LandmarkId, Landmark>& GetLandmarks() const;
  size_t NumberOfLandmarks() const;

 private:
  Map& map_;
};

class CameraView {
 public:
  CameraView(Map& map);
  size_t NumberOfCameras() const;
  geometry::Camera& GetCamera(const CameraId& cam_id);
  const std::unordered_map<CameraId, geometry::Camera>& GetCameras() const;
  bool HasCamera(const CameraId& cam_id) const;

 private:
  Map& map_;
};

class RigCameraView {
 public:
  explicit RigCameraView(Map& map);
  size_t NumberOfRigCameras() const;
  RigCamera& GetRigCamera(const RigCameraId& rig_camera_id);
  const std::unordered_map<RigCameraId, RigCamera>& GetRigCameras() const;
  bool HasRigCamera(const RigCameraId& rig_camera_id) const;

 private:
  Map& map_;
};

class RigInstanceView {
 public:
  explicit RigInstanceView(Map& map);
  size_t NumberOfRigInstances() const;
  RigInstance& GetRigInstance(const RigInstanceId& instance_id);
  const std::unordered_map<RigInstanceId, RigInstance>& GetRigInstances() const;
  bool HasRigInstance(const RigInstanceId& instance_id) const;

 private:
  Map& map_;
};

class BiasView {
 public:
  explicit BiasView(Map& map);
  size_t NumberOfBiases() const;
  geometry::Similarity& GetBias(const CameraId& cam_id);
  const std::unordered_map<CameraId, geometry::Similarity>& GetBiases() const;
  bool HasBias(const CameraId& cam_id) const;

 private:
  Map& map_;
};

}  // namespace map
