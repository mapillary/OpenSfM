#pragma once
#include <map/defines.h>

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
namespace map {
class Shot;

class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Landmark(const LandmarkId& lm_id, const Vec3d& global_pos);

  // Getters and Setters
  Vec3d GetGlobalPos() const { return global_pos_; }
  void SetGlobalPos(const Vec3d& global_pos) { global_pos_ = global_pos; }
  Vec3i GetColor() const { return color_; }
  void SetColor(const Vec3i& color) { color_ = color; }

  // Utility functions
  void AddObservation(Shot* shot, const FeatureId& feat_id) {
    observations_.emplace(shot, feat_id);
  }
  void RemoveObservation(Shot* shot);
  size_t NumberOfObservations() const { return observations_.size(); }
  FeatureId GetObservationIdInShot(Shot* shot) const {
    auto obs_it = observations_.find(shot);
    if (obs_it == observations_.end()) {
      throw std::runtime_error("Accessing with invalid shot ptr!");
    }
    return observations_.at(shot);
  }
  const std::map<Shot*, FeatureId, KeyCompare>& GetObservations() const {
    return observations_;
  }
  void ClearObservations() { observations_.clear(); }

  // Comparisons
  bool operator==(const Landmark& lm) const { return id_ == lm.id_; }
  bool operator!=(const Landmark& lm) const { return !(*this == lm); }
  bool operator<(const Landmark& lm) const { return id_ < lm.id_; }
  bool operator<=(const Landmark& lm) const { return id_ <= lm.id_; }
  bool operator>(const Landmark& lm) const { return id_ > lm.id_; }
  bool operator>=(const Landmark& lm) const { return id_ >= lm.id_; }

  // Reprojection Errors
  void SetReprojectionErrors(
      const std::map<ShotId, Eigen::VectorXd>& reproj_errors);
  std::map<ShotId, Eigen::VectorXd> GetReprojectionErrors() const {
    return reproj_errors_;
  }
  void RemoveReprojectionError(const ShotId& shot_id) {
    auto it = reproj_errors_.find(shot_id);
    if (it != reproj_errors_.end()) {
      reproj_errors_.erase(it);
    }
  }

 public:
  const LandmarkId id_;
  LandmarkUniqueId unique_id_;

 private:
  Vec3d global_pos_;  // point in global
  std::map<Shot*, FeatureId, KeyCompare> observations_;
  Vec3i color_;
  std::map<ShotId, Eigen::VectorXd> reproj_errors_;
};
}  // namespace map
