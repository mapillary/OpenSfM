#pragma once
#include <map/defines.h>
#include <Eigen/Core>
#include <unordered_map>
#include <map>
#include <memory>

#include <map/defines.h>
#include <map/pose.h>
#include <map/geo.h>
#include <sfm/tracks_manager.h>
#include <geometry/camera.h>
namespace map
{
class Shot;
class Landmark;
class ShotCamera;

class Map 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// Camera Methods
  // TODO: Removing a camera might be problematic while shots are still using it.
  // void RemoveCamera(const CameraId cam_id);
  std::vector<Camera*> GetCameras();
  Camera* GetCamera(const CameraId& cam);
  Camera* CreateCamera(const Camera& cam);
  const std::unordered_map<CameraId, Camera>& GetAllCameras() const { return cameras_; };
  const std::unordered_map<CameraId, Camera*> GetAllCameraPointers() //const
  {
    std::unordered_map<CameraId, Camera*> copy;
    //C++14
    // std::transform(cameras_.begin(), cameras_.end(), std::inserter(copy, copy.end()), [](auto& elem) { return std::make_pair(elem.first, &elem.second); });
    //C++11
    for (auto& cam_pair : cameras_)
    {
      copy.emplace(cam_pair.first, &cam_pair.second);
    }
    return copy;
  }

  // Shot Methods
  Shot* CreateShot(const ShotId shot_id, const CameraId camera_id, const Pose& pose = Pose());
  Shot* CreateShot(const ShotId shot_id, const Camera& cam, const Pose& pose = Pose());
  Shot* GetShot(const ShotId shot_id);
  void UpdateShotPose(const ShotId shot_id, const Pose& pose);
  void RemoveShot(const ShotId shot_id);
  bool HasLandmark(const LandmarkId lm_id) const { return landmarks_.count(lm_id) > 0; }
  bool HasShot(const ShotId shot_id) const { return shots_.find(shot_id) != shots_.end(); }
  const std::unordered_map<ShotId, std::unique_ptr<Shot>>& GetAllShots() const { return shots_; }
  const std::unordered_map<ShotId, Shot*> GetAllShotPointers() const
  {
    std::unordered_map<ShotId, Shot*> copy;
    //C++14
    // std::transform(shots_.begin(), shots_.end(), std::inserter(copy, copy.end()), [](auto& elem) { return std::make_pair(elem.first, elem.second.get()); });
    //C++11
    for (const auto& shot_pair : shots_)
    {
      copy.emplace(shot_pair.first, shot_pair.second.get());
    }
    return copy;
  }

  // Landmark
  Landmark* CreateLandmark(const LandmarkId lm_id, const Eigen::Vector3d& global_pos);
  Landmark* GetLandmark(const LandmarkId lm_id);
  void RemoveLandmark(const Landmark* const lm);
  void RemoveLandmark(const LandmarkId lm_id);
  void ReplaceLandmark(Landmark* old_lm, Landmark* new_lm);
  const std::unordered_map<LandmarkId, std::unique_ptr<Landmark>>& GetAllLandmarks() const { return landmarks_; };
  const std::unordered_map<LandmarkId, Landmark*> GetAllLandmarkPointers() const
  {
    std::unordered_map<LandmarkId, Landmark*> copy;
    //C++14
    // std::transform(landmarks_.begin(), landmarks_.end(), std::inserter(copy, copy.end()), [](auto& elem) { return std::make_pair(elem.first, elem.second.get()); });
    //C++11
    for (const auto& lm_pair : landmarks_)
    {
      copy.emplace(lm_pair.first, lm_pair.second.get());
    }
    return copy;
  }

  //Observation methods
  void AddObservation(Shot *const shot,  Landmark *const lm, const FeatureId feat_id);
  void AddObservation(const ShotId shot_id, const LandmarkId lm_id, const FeatureId feat_id);
  void AddObservation(Shot *const shot,  Landmark *const lm, const Observation& obs);
  void RemoveObservation(Shot *const shot,  Landmark *const lm, const FeatureId feat_id) const;
  void ClearObservationsAndLandmarks();

  // Map information and access methods
  size_t NumberOfShots() const { return shots_.size(); }
  size_t NumberOfLandmarks() const { return landmarks_.size(); }
  size_t NumberOfCameras() const { return cameras_.size(); }
  std::map<Landmark*, FeatureId> GetObservationsOfShot(const Shot* shot);
  std::map<Shot*, FeatureId> GetObservationsOfPoint(const Landmark* point);  

  // TopoCentriConverter
  const TopoCentricConverter& GetTopoCentricConverter() const
  {
    return topo_conv_;
  }

  void SetTopoCentricConverter(const double lat, const double longitude, const double alt)
  {
    topo_conv_.lat_ = lat;
    topo_conv_.long_ = longitude;
    topo_conv_.alt_ = alt;
  }


private:
  std::unordered_map<CameraId, Camera> cameras_;
  // TODO: Think about switching to objects instead of unique_ptrs
  std::unordered_map<ShotId, std::unique_ptr<Shot>> shots_;
  std::unordered_map<LandmarkId, std::unique_ptr<Landmark>> landmarks_;
  TopoCentricConverter topo_conv_;
};

} // namespace map