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

  // Should belong to the Map
  // ShotId GetShotIdFromName(const std::string& name) const { return shot_names_.at(name); }
  // LandmarkId GetPointIdFromName(const std::string& name) const { return landmark_names_.at(name); };

  // Map information
  size_t NumberOfShots() const { return shots_.size(); }
  size_t NumberOfLandmarks() const { return landmarks_.size(); }
  size_t NumberOfCameras() const { return cameras_.size(); }

  // Create, Update and Remove
  // Camera
  // ShotCamera* CreateShotCamera(const CameraId cam_id, Camera& camera, const std::string& name = "");
  void UpdateShotCamera(const CameraId cam_id, const Camera& camera);
  void RemoveShotCamera(const CameraId cam_id);

  // Shots
  Shot* CreateShot(const ShotId shot_id, const CameraId camera_id, const Pose& pose = Pose());
  Shot* CreateShot(const ShotId shot_id, const Camera& cam, const Pose& pose = Pose());
  // Shot* CreateShot(const ShotId shot_id, const ShotCamera& shot_cam, const Pose& pose = Pose());
  // Shot* CreateShot(const ShotId shot_id, const std::string& camera_id, const Pose& pose = Pose());


  void UpdateShotPose(const ShotId shot_id, const Pose& pose);
  void RemoveShot(const ShotId shot_id);
  // auto GetNextUniqueShotId() const { return unique_shot_id_; }

  Shot* GetShot(const ShotId shot_id);
  // Shot* GetShot(const std::string& shot_name);
  Landmark* GetLandmark(const LandmarkId lm_id);

  // Landmark
  Landmark* CreateLandmark(const LandmarkId lm_id, const Eigen::Vector3d& global_pos); //, const std::string& name = "");
  void UpdateLandmark(const LandmarkId lm_id, const Eigen::Vector3d& global_pos);
  void RemoveLandmark(const Landmark* const lm);
  void RemoveLandmark(const LandmarkId lm_id);
  // auto GetNextUniqueLandmarkId() const { return unique_landmark_id_; }
  void ReplaceLandmark(Landmark* old_lm, Landmark* new_lm);
  void AddObservation(Shot *const shot,  Landmark *const lm, const FeatureId feat_id);
  void AddObservation(const ShotId shot_id, const LandmarkId lm_id, const FeatureId feat_id);
  void AddObservation(Shot *const shot,  Landmark *const lm, const Observation& obs);

  void RemoveObservation(Shot *const shot,  Landmark *const lm, const FeatureId feat_id) const;

  std::map<Landmark*, FeatureId> GetObservationsOfShot(const Shot* shot);
  std::map<Shot*, FeatureId> GetObservationsOfPoint(const Landmark* point);  

  const std::unordered_map<ShotId, std::unique_ptr<Shot>>& GetAllShots() const { return shots_; }
  const std::unordered_map<CameraId, Camera>& GetAllCameras() const { return cameras_; };
  const std::unordered_map<LandmarkId, std::unique_ptr<Landmark>>& GetAllLandmarks() const { return landmarks_; };
  bool HasLandmark(const LandmarkId lm_id) const { return landmarks_.count(lm_id) > 0; }
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

  // const auto& GetAllShotNames() const
  // {
  //   return shot_names_;

  // }
  // const auto& GetAllPointNames() const
  // {
  //   return landmark_names_;
  // }

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
  // ShotCamera* GetShotCamera(const std::string& cam_name);
  // Camera* CreateCameraModel(const std::string& cam_name, const Camera& cam);
  bool HasShot(const ShotId shot_id) const { return shots_.find(shot_id) != shots_.end(); }
  void ClearObservationsAndLandmarks();

  std::vector<Camera*> GetAllCameraModels();
  Camera* GetCamera(const CameraId& cam);
  Camera* CreateCamera(const Camera& cam);
private:
  std::unordered_map<CameraId, Camera> cameras_;
  // std::unordered_map<CameraId, std::unique_ptr<Camera>> cameras_;
  std::unordered_map<ShotId, std::unique_ptr<Shot>> shots_;
  std::unordered_map<LandmarkId, std::unique_ptr<Landmark>> landmarks_;

  // ShotUniqueId shot_unique_id_ = 0;
  // LandmarkUniqueId landmark_unique_id_ = 0;
  TopoCentricConverter topo_conv_;
};

} // namespace map