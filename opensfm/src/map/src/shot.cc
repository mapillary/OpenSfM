#include <map/shot.h>
#include <map/landmark.h>
#include <algorithm>
#include <numeric>
namespace map
{
ShotUniqueId Shot::shot_unique_id_ = 0;
// Shot::Shot(const ShotId shot_id, const ShotCamera& shot_camera, const Pose& pose):
//             id_(shot_id), unique_id_(shot_unique_id_), shot_camera_(shot_camera), slam_data_(this), pose_(pose)
// {
//   ++Shot::shot_unique_id_;
// }
Shot::Shot(const ShotId shot_id, const Camera& shot_camera, const Pose& pose):
            id_(shot_id), unique_id_(shot_unique_id_), shot_camera_(shot_camera), slam_data_(this), pose_(pose)
{
  ++Shot::shot_unique_id_;
}

size_t
Shot::ComputeNumValidLandmarks(const int min_obs_thr) const
{
  return std::accumulate(landmarks_.cbegin(), landmarks_.cend(), 0,
                    [min_obs_thr](const size_t prior, const Landmark* lm)
                    {
                        if (lm != nullptr && min_obs_thr <= lm->NumberOfObservations())
                          return prior + 1;
                        return prior;
                    });
}

float
Shot::ComputeMedianDepthOfLandmarks(const bool take_abs) const
{
  if (landmarks_.empty())
    return 1.0f;
  std::vector<float> depths;
  depths.reserve(landmarks_.size());
  const Eigen::Matrix4d T_cw = pose_.WorldToCamera();
  const Eigen::Vector3d rot_cw_z_row = T_cw.block<1, 3>(2, 0);

  // T_cw
  const double trans_cw_z = T_cw(2, 3);
  for (const auto& lm : landmarks_)
  {
      if (lm != nullptr)
      {
        const double pos_c_z = rot_cw_z_row.dot(lm->GetGlobalPos())+trans_cw_z;
        depths.push_back(float(take_abs ? std::abs(pos_c_z) : pos_c_z));
      }
  }
  std::sort(depths.begin(), depths.end());
  return depths.at((depths.size() - 1) / 2);
}

void
Shot::InitKeyptsAndDescriptors(const size_t n_keypts)
{
  if (n_keypts > 0)
  {
    num_keypts_ = n_keypts;
    landmarks_.resize(num_keypts_, nullptr);
    keypoints_.resize(num_keypts_);
    // descriptors_ = cv::Mat(n_keypts, 32, CV_8UC1, cv::Scalar(0));
    descriptors_ = DescriptorMatrix(n_keypts, 32);
  }
}

void
// Shot::InitAndTakeDatastructures(AlignedVector<Observation> keypts, cv::Mat descriptors)
Shot::InitAndTakeDatastructures(AlignedVector<Observation> keypts, DescriptorMatrix descriptors)
{
  assert(keypts.size() == descriptors.rows());

  std::swap(keypts, keypoints_);
  std::swap(descriptors, descriptors_);
  num_keypts_ = keypoints_.size();
  landmarks_.resize(num_keypts_, nullptr);
}

// void
// Shot::UndistortedKeyptsToBearings()
// {
//   if (!slam_data_.undist_keypts_.empty())
//   {
    
//     shot_camera_.camera_model_.UndistortedKeyptsToBearings(slam_data_.undist_keypts_, slam_data_.bearings_);
//   }
// }

// void
// Shot::UndistortKeypts()
// {
//   if (!keypoints_.empty())
//   {
//     shot_camera_.camera_model_.UndistortKeypts(keypoints_, slam_data_.undist_keypts_);
//   }
// }

void
Shot::ScaleLandmarks(const float scale)
{
  for (auto lm : landmarks_) 
  {
    if (lm != nullptr)
    {
      lm->SetGlobalPos(lm->GetGlobalPos()*scale);
    }
  }
}

void
Shot::ScalePose(const float scale)
{
    Eigen::Matrix4d cam_pose_cw = pose_.WorldToCamera();
    cam_pose_cw.block<3, 1>(0, 3) *= scale;
    pose_.SetFromWorldToCamera(cam_pose_cw);
}

void 
Shot::RemoveLandmarkObservation(const FeatureId id) 
{ 
  if (landmarks_.empty())
  {
    auto* lm = landmark_id_.at(id);
    // std::cout << "Got: " << lm->id_ << ", " << landmark_id_.size() << "/" << landmark_observations_.size() 
              // << " from shot: " << id_ << ", " << name_ << std::endl;
    // std::cout << "Erasing: " << lm << " and " << id << std::endl;
    // if (landmark_id_.find(id) != landmark_id_.end())
    // {
    //   std::cout << "found id" << std::endl;
    // }
    // if (landmark_observations_.find(lm) != landmark_observations_.end())
    // {
    //   std::cout << "found lm" << std::endl;
    // }
    landmark_id_.erase(id);
    landmark_observations_.erase(lm);
  }
  else
  {
    landmarks_.at(id) = nullptr; 
  }
}

Eigen::Vector2d Shot::Project(const Eigen::Vector3d& global_pos) const {
  // TODO: Think about oob reprojeciotn
  // Eigen::Vector2d new_pt;
  //rotate
  // const Eigen::Vector3d rot_pt = pose_.RotationWorldToCamera()*global_pos + pose_.TranslationWorldToCamera();
  return shot_camera_.Project(pose_.RotationWorldToCamera()*global_pos + pose_.TranslationWorldToCamera());
  // shot_camera_.camera_model_.ReprojectToImage(pose_.RotationWorldToCamera(),
                                              // pose_.TranslationWorldToCamera(),
                                              // global_pos, new_pt);
  // return new_pt;
}

} //namespace map

