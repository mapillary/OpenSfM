#pragma once
#include <Eigen/Eigen>
#include <map>
#include <unordered_map>
#include <memory>
#include <map/defines.h>
#include <iostream>
// #include <map/shot.h>
namespace map
{
class Shot;

class SLAMLandmarkData{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // cv::Mat descriptor_;
  DescriptorType descriptor_;
  size_t num_observations_ = 0;

  Vec3d mean_normal_ = Vec3d::Zero();
  float GetMinValidDistance() const { return 0.7 * min_valid_dist_; }
  float GetMaxValidDistance() const { return 1.3 * max_valid_dist_; }
  void IncreaseNumObservable(unsigned int num_observable = 1){ num_observable_ += num_observable; }
  void IncreaseNumObserved(unsigned int num_observed = 1) { num_observed_ += num_observed; }
  float GetObservedRatio() const { return static_cast<float>(num_observed_)/num_observable_; }
  // ORB scale variances
  //! max valid distance between landmark and camera
  float min_valid_dist_ = 0;
  //! min valid distance between landmark and camera
  float max_valid_dist_ = 0; 
  size_t GetNumObserved() const { return num_observed_; }
  size_t GetNumObservable() const { return num_observable_; }
private:
  // track counter
  size_t num_observable_ = 1;
  size_t num_observed_ = 1;
};

class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Landmark(const LandmarkId& lm_id, const Vec3d& global_pos);

  // Getters and Setters
  Vec3d GetGlobalPos() const { return global_pos_; }
  void SetGlobalPos(const Vec3d& global_pos) { global_pos_ = global_pos; }
  Vec3i GetColor() const { return color_; }
  void SetColor(const Vec3i& color) { color_ = color; }
  void SetRefShot(Shot* ref_shot) { ref_shot_ = ref_shot; }
  Shot* GetRefShot() { return ref_shot_; }

  // Utility functions
  bool IsObservedInShot(Shot* shot) const { return observations_.count(shot) > 0; }
  void AddObservation(Shot* shot, const FeatureId feat_id) { observations_.emplace(shot, feat_id); }
  void RemoveObservation(Shot* shot);
  bool HasObservations() const { return !observations_.empty(); }
  size_t NumberOfObservations() const { return observations_.size(); }
  Vec3f GetObservationInShot(Shot* shot) const;
  FeatureId GetObservationIdInShot(Shot* shot) const
  {
    return observations_.at(shot);
  }
  const std::map<Shot*, FeatureId, KeyCompare>& GetObservations() const {
    return observations_;
  }
  void ClearObservations() { observations_.clear(); }
  double ComputeDistanceFromRefFrame() const;

  //Comparisons
  bool operator==(const Landmark& lm) const { return id_ == lm.id_; }
  bool operator!=(const Landmark& lm) const { return !(*this == lm); }
  bool operator<(const Landmark& lm) const { return id_ < lm.id_; }
  bool operator<=(const Landmark& lm) const { return id_ <= lm.id_; }
  bool operator>(const Landmark& lm) const { return id_ > lm.id_; }
  bool operator>=(const Landmark& lm) const { return id_ >= lm.id_; }

  // Reprojection Errors
  void SetReprojectionErrors(const std::unordered_map<ShotId, Eigen::VectorXd> reproj_errors);
  std::unordered_map<ShotId, Eigen::VectorXd> GetReprojectionErrors() const { return reproj_errors_; }
  void RemoveReprojectionError(const ShotId& shot_id)
  {
    auto it = reproj_errors_.find(shot_id);
    if (it != reproj_errors_.end())
    {
      reproj_errors_.erase(it);
    }
  }

public:
  const LandmarkId id_;
  LandmarkUniqueId unique_id_;
  SLAMLandmarkData slam_data_;
private:
  Vec3d global_pos_; // point in global
  std::map<Shot*, FeatureId, KeyCompare> observations_;
  Shot* ref_shot_; //shot in which the landmark was first seen
  Vec3i color_;
  std::unordered_map<ShotId, Eigen::VectorXd> reproj_errors_;

};
}