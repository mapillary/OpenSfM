#include <map/shot.h>
#include <map/landmark.h>
#include <algorithm>
#include <numeric>
namespace map
{

Shot::Shot(const ShotId& shot_id, const Camera* const shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      shot_camera_(shot_camera),
      slam_data_(this),
      pose_(pose),
      merge_cc(0),
      scale(0) {}

Shot::Shot(const ShotId& shot_id, std::unique_ptr<Camera> shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      shot_camera_(shot_camera.get()),
      slam_data_(this),
      pose_(pose),
      merge_cc(0),
      scale(0) {
  own_camera_ = std::move(shot_camera);
}

void
ShotMeasurements::Set(const ShotMeasurements& other)
{
  // ShotMeasurement<double> capture_time_;
  if (other.capture_time_.HasValue())
  {
    capture_time_.SetValue(other.capture_time_.Value());
  }
  // ShotMeasurement<Vec3d> gps_position_;
  if (other.gps_position_.HasValue())
  {
    gps_position_.SetValue(other.gps_position_.Value());
  }
  // ShotMeasurement<double> gps_accuracy_;
  if (other.gps_accuracy_.HasValue())
  {
    gps_accuracy_.SetValue(other.gps_accuracy_.Value());
  }
  // ShotMeasurement<double> compass_accuracy_;
  if (other.compass_accuracy_.HasValue())
  {
    compass_accuracy_.SetValue(other.compass_accuracy_.Value());
  }
  // ShotMeasurement<double> compass_angle_;
  if (other.compass_angle_.HasValue())
  {
    compass_angle_.SetValue(other.compass_angle_.Value());
  }
  // ShotMeasurement<Vec3d> accelerometer_;
  if (other.accelerometer_.HasValue())
  {
    accelerometer_.SetValue(other.accelerometer_.Value());
  }
  // ShotMeasurement<int> orientation_;
  if (other.orientation_.HasValue())
  {
    orientation_.SetValue(other.orientation_.Value());
  }
  // ShotMeasurement<std::string> sequence_key_;
  if (other.sequence_key_.HasValue())
  {
    sequence_key_.SetValue(other.sequence_key_.Value());
  }
}

size_t
Shot::ComputeNumValidLandmarks(const int min_obs_thr) const
{
  if (landmarks_.empty())
  {
      return std::accumulate(landmark_observations_.cbegin(), landmark_observations_.cend(), 0,
                    [min_obs_thr](const size_t prior, const std::pair<Landmark*, Observation>& lm)
                    {
                        if (min_obs_thr <= lm.first->NumberOfObservations())
                          return prior + 1;
                        return prior;
                    });
  }
  else
  {
    return std::accumulate(landmarks_.cbegin(), landmarks_.cend(), 0,
                    [min_obs_thr](const size_t prior, const Landmark* lm)
                    {
                        if (lm != nullptr && min_obs_thr <= lm->NumberOfObservations())
                          return prior + 1;
                        return prior;
                    });
  }
}

float
Shot::ComputeMedianDepthOfLandmarks(const bool take_abs) const
{
  std::vector<float> depths;
  const auto n_landmarks = UseLinearDataStructure() ? landmarks_.size() : landmark_observations_.size();
  depths.reserve(n_landmarks);
  const Mat4d T_cw = pose_.WorldToCamera();
  const Vec3d rot_cw_z_row = T_cw.block<1, 3>(2, 0);
  const double trans_cw_z = T_cw(2, 3);
  if (UseLinearDataStructure())
  {
    for (const auto& lm : landmarks_)
    {
        if (lm != nullptr)
        {
          const double pos_c_z = rot_cw_z_row.dot(lm->GetGlobalPos())+trans_cw_z;
          depths.push_back(float(take_abs ? std::abs(pos_c_z) : pos_c_z));
        }
    }
  }
  else
  {
    for (const auto& lm_pair : landmark_observations_)
    {
      auto* lm = lm_pair.first;
      const double pos_c_z = rot_cw_z_row.dot(lm->GetGlobalPos())+trans_cw_z;
      depths.push_back(float(take_abs ? std::abs(pos_c_z) : pos_c_z));
    }
  }
  if (depths.empty())
  {
    return 0;
  }
  else
  {
    std::sort(depths.begin(), depths.end());
    return depths.at((depths.size() - 1) / 2);
  }
}

void
Shot::InitKeyptsAndDescriptors(const size_t n_keypts)
{
  if (n_keypts > 0)
  {
    num_keypts_ = n_keypts;
    landmarks_.resize(num_keypts_, nullptr);
    keypoints_.resize(num_keypts_);
    descriptors_ = DescriptorMatrix(n_keypts, 32);
  }
}

void
Shot::InitAndTakeDatastructures(AlignedVector<Observation> keypts, DescriptorMatrix descriptors)
{
  assert(keypts.size() == descriptors.rows());

  std::swap(keypts, keypoints_);
  std::swap(descriptors, descriptors_);
  num_keypts_ = keypoints_.size();
  landmarks_.resize(num_keypts_, nullptr);
}

void
Shot::NormalizeKeypts()
{
  const auto& cam = shot_camera_;

  auto normalize_obseration = [&cam](Observation& obs)
  {
    const Vec3d norm_pt = cam->NormalizeImageCoordinateAndScale(Vec3d(obs.point[0], obs.point[1], obs.scale));
    obs.point = norm_pt.head<2>();
    obs.scale = norm_pt[2];
  };

  if (UseLinearDataStructure())
  {
    for (auto& obs : keypoints_)
    {
      normalize_obseration(obs);
    }
  }
  else
  {
    for (auto& lm_obs : landmark_observations_)
    {
      auto& obs = lm_obs.second;
      normalize_obseration(obs);
    }
  }
}

//For SLAM
void Shot::UndistortAndComputeBearings() {
  slam_data_.bearings_.reserve(keypoints_.size());
  slam_data_.undist_keypts_.reserve(keypoints_.size());

  for (size_t idx = 0; idx < keypoints_.size(); ++idx) {
    slam_data_.bearings_.push_back(shot_camera_->Bearing(shot_camera_->NormalizeImageCoordinate(keypoints_[idx].point)));
    slam_data_.undist_keypts_.emplace_back(keypoints_.at(idx));
    
    auto& obs = slam_data_.undist_keypts_.back();
    obs.point = shot_camera_->Project(shot_camera_->Bearing(keypoints_[idx].point));
  }
  std::cout << id_ << "slam_data_.undist_keypts_: " << slam_data_.undist_keypts_.size() << std::endl;
}

void
Shot::ScaleLandmarks(const double scale)
{
  if (UseLinearDataStructure())
  {
    for (auto* lm : landmarks_) 
    {
      if (lm != nullptr)
      {
        lm->SetGlobalPos(lm->GetGlobalPos()*scale);
      }
    }
  }
  else
  {
    for (auto& lm_obs : landmark_observations_)
    {
      auto* lm = lm_obs.first;
      lm->SetGlobalPos(lm->GetGlobalPos()*scale);
    }
  }
}

void
Shot::IncreaseObservedOfLandmarks()
{
  if (UseLinearDataStructure()) // for SLAM
  {
    for (auto* lm : landmarks_)
    {
      if (lm != nullptr) 
      {
        lm->slam_data_.IncreaseNumObserved();
      }
    }
  }
  else // for OpenSfM
  {
    for (auto& lm_pair : landmark_observations_)
    {
      lm_pair.first->slam_data_.IncreaseNumObserved();
    }
  }
}

void
Shot::ScalePose(const double scale)
{
    Mat4d cam_pose_cw = pose_.WorldToCamera();
    cam_pose_cw.block<3, 1>(0, 3) *= scale;
    pose_.SetFromWorldToCamera(cam_pose_cw);
}

void 
Shot::RemoveLandmarkObservation(const FeatureId id) 
{
  if (UseLinearDataStructure()) // for SLAM
  {
    landmarks_.at(id) = nullptr; 
  }
  else // for OpenSfM
  {
    auto* lm = landmark_id_.at(id);
    landmark_id_.erase(id);
    landmark_observations_.erase(lm);
  }
}

Vec2d Shot::Project(const Vec3d& global_pos) const {
  return shot_camera_->Project(pose_.RotationWorldToCamera()*global_pos + pose_.TranslationWorldToCamera());
}


Vec2d Shot::ProjectInImageCoordinates(const Vec3d& global_pos) const {
  const Vec3d pt = pose_.RotationWorldToCamera()*global_pos + pose_.TranslationWorldToCamera(); 
  // if ((shot_camera_->GetProjectionMatrixScaled()*pt)[2] < 0)
    // std::cout << "out of bounds" << (shot_camera_->GetProjectionMatrixScaled()*pt).hnormalized().transpose() << std::endl;
  return (shot_camera_->GetProjectionMatrixScaled()*pt).hnormalized();
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

} //namespace map

