#pragma once
// #include <map/map.h>
#include <geometry/camera.h>
#include <map/landmark.h>
#include <map/shot.h>

#include <unordered_map>
namespace map {
class Map;
class ShotView {
 public:
  ShotView(Map& map);
  Shot* GetShot(const map::ShotId& shot_id);
  bool HasShot(const map::ShotId& shot_id);
  const std::unordered_map<ShotId, Shot>& GetShots() const;
  size_t NumberOfShots() const;

 private:
  Map& map_;
};

class PanoShotView {
 public:
  PanoShotView(Map& map);
  Shot* GetShot(const map::ShotId& shot_id);
  bool HasShot(const map::ShotId& shot_id);
  const std::unordered_map<ShotId, Shot>& GetShots() const;
  size_t NumberOfShots() const;
 private:
  Map& map_;
};

class LandmarkView {
 public:
  LandmarkView(Map& map);
  Landmark* GetLandmark(const LandmarkId& lm_id);
  bool HasLandmark(const LandmarkId& lm_id);
  const std::unordered_map<LandmarkId, Landmark>&
  GetLandmarks() const;
  size_t NumberOfLandmarks() const;

 private:
  Map& map_;
};

class CameraView {
 public:
  CameraView(Map& map);
  size_t NumberOfCameras() const;
  Camera* GetCamera(const CameraId& cam_id);
  const std::unordered_map<CameraId, Camera>& GetCameras() const;
  bool HasCamera(const CameraId& cam_id) const;

 private:
  Map& map_;
};

}  // namespace map
