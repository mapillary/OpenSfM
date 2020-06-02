#pragma once
#include <map/map.h>
namespace map {
class ShotView {
 public:
  ShotView(Map& map) : map_(map) {}
  Shot* GetShot(const map::ShotId& shot_id) { return map_.GetShot(shot_id); }
  bool HasShot(const map::ShotId& shot_id) { return map_.HasShot(shot_id); }
  const std::unordered_map<ShotId, std::unique_ptr<Shot>>& GetShots() const {
    return map_.GetAllShots();
  }
  size_t NumberOfShots() const { return map_.NumberOfShots();}
 private:
  Map& map_;
};

class LandmarkView {
 public:
  LandmarkView(Map& map) : map_(map) {}
  Landmark* GetLandmark(const LandmarkId& lm_id) { return map_.GetLandmark(lm_id); }
  bool HasLandmark(const LandmarkId& lm_id) { return map_.HasLandmark(lm_id); }
  const std::unordered_map<LandmarkId, std::unique_ptr<Landmark>>&
  GetLandmarks() const {
    return map_.GetAllLandmarks();
  }
  size_t NumberOfLandmarks() const { return map_.NumberOfLandmarks();}

 private:
  Map& map_;
};

class CameraView {
 public:
  CameraView(Map& map) : map_(map) {}
  size_t NumberOfCameras() const { return map_.NumberOfCameras(); }
  Camera* GetCamera(const CameraId& cam_id) { return map_.GetCamera(cam_id);}
  const std::unordered_map<CameraId, Camera>&
  GetCameras() const { return map_.GetAllCameras(); }
  bool HasCamera(const CameraId& cam_id) const { return map_.HasCamera(cam_id);}
 private:
  Map& map_;
};

}  // namespace map
