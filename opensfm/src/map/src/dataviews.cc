// #pragma once
#include <map/map.h>

namespace map {
  ShotView::ShotView(Map& map) : map_(map) {}
  Shot* ShotView::GetShot(const map::ShotId& shot_id) { return map_.GetShot(shot_id); }
  bool ShotView::HasShot(const map::ShotId& shot_id) { return map_.HasShot(shot_id); }
  const std::unordered_map<ShotId, std::unique_ptr<Shot>>& ShotView::GetShots() const {
    return map_.GetAllShots();
  }
  size_t ShotView::NumberOfShots() const { return map_.NumberOfShots();}

  LandmarkView::LandmarkView(Map& map) : map_(map) {}
  Landmark* LandmarkView::GetLandmark(const LandmarkId& lm_id) { return map_.GetLandmark(lm_id); }
  bool LandmarkView::HasLandmark(const LandmarkId& lm_id) { return map_.HasLandmark(lm_id); }
  const std::unordered_map<LandmarkId, std::unique_ptr<Landmark>>&
  LandmarkView::GetLandmarks() const {
    return map_.GetAllLandmarks();
  }
  size_t LandmarkView::NumberOfLandmarks() const { return map_.NumberOfLandmarks();}

  CameraView::CameraView(Map& map) : map_(map) {}
  size_t CameraView::NumberOfCameras() const { return map_.NumberOfCameras(); }
  Camera* CameraView::GetCamera(const CameraId& cam_id) { return map_.GetCamera(cam_id);}
  const std::unordered_map<CameraId, Camera>&
  CameraView::GetCameras() const { return map_.GetAllCameras(); }
  bool CameraView::HasCamera(const CameraId& cam_id) const { return map_.HasCamera(cam_id);}

}  // namespace map
