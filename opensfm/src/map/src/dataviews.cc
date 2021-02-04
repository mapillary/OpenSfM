#include <map/map.h>

namespace map {
ShotView::ShotView(Map& map) : map_(map) {}
Shot& ShotView::GetShot(const map::ShotId& shot_id) {
  return map_.GetShot(shot_id);
}
bool ShotView::HasShot(const map::ShotId& shot_id) const {
  return map_.HasShot(shot_id);
}
const std::unordered_map<ShotId, Shot>& ShotView::GetShots() const {
  return map_.GetShots();
}
size_t ShotView::NumberOfShots() const { return map_.NumberOfShots(); }

PanoShotView::PanoShotView(Map& map) : map_(map) {}
Shot& PanoShotView::GetShot(const map::ShotId& shot_id) {
  return map_.GetPanoShot(shot_id);
}
bool PanoShotView::HasShot(const map::ShotId& shot_id) const {
  return map_.HasPanoShot(shot_id);
}
const std::unordered_map<ShotId, Shot>& PanoShotView::GetShots() const {
  return map_.GetPanoShots();
}
size_t PanoShotView::NumberOfShots() const { return map_.NumberOfPanoShots(); }

LandmarkView::LandmarkView(Map& map) : map_(map) {}
Landmark& LandmarkView::GetLandmark(const LandmarkId& lm_id) {
  return map_.GetLandmark(lm_id);
}
bool LandmarkView::HasLandmark(const LandmarkId& lm_id) const {
  return map_.HasLandmark(lm_id);
}
const std::unordered_map<LandmarkId, Landmark>& LandmarkView::GetLandmarks()
    const {
  return map_.GetLandmarks();
}
size_t LandmarkView::NumberOfLandmarks() const {
  return map_.NumberOfLandmarks();
}

CameraView::CameraView(Map& map) : map_(map) {}
size_t CameraView::NumberOfCameras() const { return map_.NumberOfCameras(); }
Camera& CameraView::GetCamera(const CameraId& cam_id) {
  return map_.GetCamera(cam_id);
}
const std::unordered_map<CameraId, Camera>& CameraView::GetCameras() const {
  return map_.GetCameras();
}
bool CameraView::HasCamera(const CameraId& cam_id) const {
  return map_.HasCamera(cam_id);
}

}  // namespace map
