#include <map/shot.h>
#include <map/landmark.h>
#include <algorithm>
#include <numeric>
namespace map {

Shot::Shot(const ShotId& shot_id, const Camera* const shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      shot_camera_(shot_camera),
      slam_data_(this),
      merge_cc(0),
      scale(1.0),
      pose_(pose) {}

Shot::Shot(const ShotId& shot_id, std::unique_ptr<Camera> shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      shot_camera_(shot_camera.get()),
      slam_data_(this),
      merge_cc(0),
      scale(1.0),
      pose_(pose) {
  own_camera_ = std::move(shot_camera);
}

void ShotMeasurements::Set(const ShotMeasurements& other) {
  if (other.capture_time_.HasValue()) {
    capture_time_.SetValue(other.capture_time_.Value());
  } else {
    capture_time_.Reset();
  }
  if (other.gps_position_.HasValue()) {
    gps_position_.SetValue(other.gps_position_.Value());
  } else {
    gps_position_.Reset();
  }
  if (other.gps_accuracy_.HasValue()) {
    gps_accuracy_.SetValue(other.gps_accuracy_.Value());
  } else {
    gps_accuracy_.Reset();
  }
  if (other.compass_accuracy_.HasValue()) {
    compass_accuracy_.SetValue(other.compass_accuracy_.Value());
  } else {
    compass_accuracy_.Reset();
  }

  if (other.compass_angle_.HasValue()) {
    compass_angle_.SetValue(other.compass_angle_.Value());
  } else {
    compass_angle_.Reset();
  }
  if (other.accelerometer_.HasValue()) {
    accelerometer_.SetValue(other.accelerometer_.Value());
  } else {
    accelerometer_.Reset();
  }
  if (other.orientation_.HasValue()) {
    orientation_.SetValue(other.orientation_.Value());
  } else {
    orientation_.Reset();
  }
  if (other.sequence_key_.HasValue()) {
    sequence_key_.SetValue(other.sequence_key_.Value());
  } else {
    sequence_key_.Reset();
  }
}

size_t Shot::ComputeNumValidLandmarks(const int min_obs_thr) const {
  if (UseLinearDataStructure()) {
    return std::accumulate(
        landmarks_.cbegin(), landmarks_.cend(), 0,
        [min_obs_thr](const size_t prior, const Landmark* lm) {
          if (lm != nullptr && min_obs_thr <= lm->NumberOfObservations())
            return prior + 1;
          return prior;
        });
  } else {
    return std::accumulate(
        landmark_observations_.cbegin(), landmark_observations_.cend(), 0,
        [min_obs_thr](const size_t prior,
                      const std::pair<Landmark*, Observation>& lm) {
          if (min_obs_thr <= lm.first->NumberOfObservations()) return prior + 1;
          return prior;
        });
  }
}

float Shot::ComputeMedianDepthOfLandmarks(const bool take_abs) const {
  std::vector<float> depths;
  depths.reserve(landmarks_.size());
  const Mat4d T_cw = pose_.WorldToCamera();
  const Vec3d rot_cw_z_row = T_cw.block<1, 3>(2, 0);
  const double trans_cw_z = T_cw(2, 3);
  if (UseLinearDataStructure()) {
    for (const auto& lm : landmarks_) {
      if (lm != nullptr) {
        const double pos_c_z =
            rot_cw_z_row.dot(lm->GetGlobalPos()) + trans_cw_z;
        depths.push_back(float(take_abs ? std::abs(pos_c_z) : pos_c_z));
      }
    }
  } else {
    for (const auto& lm_pair : landmark_observations_) {
      auto* lm = lm_pair.first;
      const double pos_c_z = rot_cw_z_row.dot(lm->GetGlobalPos()) + trans_cw_z;
      depths.push_back(float(take_abs ? std::abs(pos_c_z) : pos_c_z));
    }
  }
  if (depths.empty()) {
    return 0;
  }
  std::sort(depths.begin(), depths.end());
  return depths.at((depths.size() - 1) / 2);
}

void Shot::InitKeyptsAndDescriptors(const size_t n_keypts) {
  if (n_keypts > 0) {
    num_keypts_ = n_keypts;
    landmarks_.resize(num_keypts_, nullptr);
    keypoints_.resize(num_keypts_);
    descriptors_ = DescriptorMatrix(n_keypts, 32);
  }
}

void Shot::InitAndTakeDatastructures(AlignedVector<Observation> keypts,
                                     DescriptorMatrix descriptors) {
  assert(keypts.size() == descriptors.rows());

  std::swap(keypts, keypoints_);
  std::swap(descriptors, descriptors_);
  num_keypts_ = keypoints_.size();
  landmarks_.resize(num_keypts_, nullptr);
}

void Shot::ScaleLandmarks(const double scale) {
  if (UseLinearDataStructure()) {
    for (auto* lm : landmarks_) {
      if (lm != nullptr) {
        lm->SetGlobalPos(lm->GetGlobalPos() * scale);
      }
    }
  } else {
    for (auto& lm_obs : landmark_observations_) {
      auto* lm = lm_obs.first;
      lm->SetGlobalPos(lm->GetGlobalPos() * scale);
    }
  }
}

void Shot::ScalePose(const double scale) {
  Mat4d cam_pose_cw = pose_.WorldToCamera();
  cam_pose_cw.block<3, 1>(0, 3) *= scale;
  pose_.SetFromWorldToCamera(cam_pose_cw);
}

void Shot::RemoveLandmarkObservation(const FeatureId id) {
  // for SLAM
  if (UseLinearDataStructure()) {
    landmarks_.at(id) = nullptr;
  } else  // for OpenSfM
  {
    auto* lm = landmark_id_.at(id);
    landmark_id_.erase(id);
    landmark_observations_.erase(lm);
  }
}

Vec2d Shot::Project(const Vec3d& global_pos) const {
  return shot_camera_->Project(pose_.RotationWorldToCamera() * global_pos +
                               pose_.TranslationWorldToCamera());
}

MatX2d Shot::ProjectMany(const MatX3d& points) const {
  MatX2d projected(points.rows(), 2);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Project(points.row(i));
  }
  return projected;
}

Vec3d Shot::Bearing(const Vec2d& point) const {
  return pose_.RotationCameraToWorld() * shot_camera_->Bearing(point);
}

MatX3d Shot::BearingMany(const MatX2d& points) const {
  MatX3d bearings(points.rows(), 3);
  for (int i = 0; i < points.rows(); ++i) {
    bearings.row(i) = Bearing(points.row(i));
  }
  return bearings;
}

}  // namespace map
