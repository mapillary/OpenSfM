#pragma once
#include <geometry/camera.h>
#include <map/defines.h>
#include <map/landmark.h>
#include <map/pose.h>
#include <sfm/observation.h>

#include <Eigen/Eigen>
#include <unordered_map>

namespace map {
class Pose;
class Map;
class SLAMShotData {
 public:
  SLAMShotData() = delete;
  SLAMShotData(Shot* shot) {
  }  //:graph_node_(std::make_unique<data::graph_node>(shot, false)){}
  AlignedVector<Observation> undist_keypts_;
  AlignedVector<Eigen::Vector3d> bearings_;
  std::vector<std::vector<std::vector<size_t>>> keypt_indices_in_cells_;
  // const std::unique_ptr<data::graph_node> graph_node_ = nullptr;
  // void UpdateGraphNode()
  // {
  //   graph_node_->update_connections();
  // }
};

struct ShotMesh {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void SetVertices(const Eigen::MatrixXd& vertices) { vertices_ = vertices; }
  void SetFaces(const Eigen::MatrixXd& faces) { faces_ = faces; }
  Eigen::MatrixXd GetFaces() const { return faces_; }
  Eigen::MatrixXd GetVertices() const { return vertices_; }
  Eigen::MatrixXd vertices_;
  Eigen::MatrixXd faces_;
};

struct ShotMeasurements {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double capture_time_;
  // TODO: Correct data types for compass,....
  double compass_;
  double accelerometer_;
  double gps_dop_{0};
  // std::array<double, 3> gps_position_{0};
  Vec3d gps_position_;
  int orientation_;
  std::string skey_;
};

class Shot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Shot(const ShotId& shot_id, const Camera* const shot_camera, const Pose& pose);
  // Workaround for pickle that makes it possible for the shot to have camera
  // outside of the reconstruction.
  Shot(const ShotId& shot_id, std::unique_ptr<Camera> cam, const Pose& pose);
  

  const DescriptorType GetDescriptor(const FeatureId id) const {
    return descriptors_.row(id);
  }
 
  Eigen::Vector3f GetKeyPointEigen(const FeatureId id) const {
    const auto kpt = keypoints_.at(id);
    return Eigen::Vector3f(kpt.point[0], kpt.point[1], kpt.size);
  }
  // No reason to set individual keypoints or descriptors
  // read-only access
  const AlignedVector<Observation>& GetKeyPoints() const { return keypoints_; }
  const DescriptorMatrix& GetDescriptors() const { return descriptors_; }

  // size_t NumberOfKeyPoints() const { return keypoints_.size(); }
  size_t ComputeNumValidLandmarks(const int min_obs_thr = 1) const;
  float ComputeMedianDepthOfLandmarks(const bool take_abs) const;

  const std::vector<Landmark*>& GetLandmarks() const { return landmarks_; }
  std::vector<Landmark*>& GetLandmarks() { return landmarks_; }

  const Observation& GetObservation(const FeatureId id) const {
    return landmarks_.empty() ? landmark_observations_.at(landmark_id_.at(id))
                              : keypoints_.at(id);
  }

//  const Observation& GetKeyPoint(const FeatureId id) const {
//     return keypoints_.at(id);
//   }

  std::vector<Landmark*> ComputeValidLandmarks() {
    std::vector<Landmark*> valid_landmarks;

    // we use the landmark observation
    if (landmarks_.empty()) {
      valid_landmarks.reserve(landmark_observations_.size());
      // C++ 14
      // std::transform(landmark_observations_.begin(),
      // landmark_observations_.end(),
      //           std::back_inserter(valid_landmarks), [](const auto& p){
      //           return p.first; });
      for (const auto& lm_obs : landmark_observations_) {
        valid_landmarks.push_back(lm_obs.first);
      }
    } else {
      valid_landmarks.reserve(landmarks_.size());
      std::copy_if(landmarks_.begin(), landmarks_.end(),
                   std::back_inserter(valid_landmarks),
                   [](const Landmark* lm) { return lm != nullptr; });
    }
    return valid_landmarks;
  }

  // std::vector<FeatureId> ComputeValidLandmarksIndices() const {
  //   std::vector<FeatureId> valid_landmarks;
  //   valid_landmarks.reserve(landmarks_.size());
  //   for (size_t idx = 0; idx < landmarks_.size(); ++idx) {
  //     if (landmarks_[idx] != nullptr) {
  //       valid_landmarks.push_back(idx);
  //     }
  //   }
  //   return valid_landmarks;
  // }

  // std::vector<std::pair<Landmark*, FeatureId>> ComputeValidLandmarksAndIndices()
  //     const {
  //   std::vector<std::pair<Landmark*, FeatureId>> valid_landmarks;
  //   valid_landmarks.reserve(landmarks_.size());
  //   for (size_t idx = 0; idx < landmarks_.size(); ++idx) {
  //     auto* lm = landmarks_[idx];
  //     if (lm != nullptr) {
  //       valid_landmarks.push_back(std::make_pair(lm, idx));
  //     }
  //   }
  //   return valid_landmarks;
  // }

  Landmark* GetLandmark(const FeatureId id) { return landmarks_.at(id); }
  void RemoveLandmarkObservation(const FeatureId id);
  void AddLandmarkObservation(Landmark* lm, const FeatureId feat_id) {
    landmarks_.at(feat_id) = lm;
  }

  void CreateObservation(Landmark* lm, const Vec2d& pt,
                         const double scale, const Eigen::Vector3i& color,
                         FeatureId id) {
    // C++ 14
    // landmark_observations_.insert(std::make_pair(lm,
    // std::make_unique<Observation>(pt[0], pt[1], scale, color[0], color[1],
    // color[2], id)));
    landmark_observations_.insert(std::make_pair(
        lm,
        Observation(pt[0], pt[1], scale, color[0], color[1], color[2], id)));
    landmark_id_.insert(std::make_pair(id, lm));
  }

  Observation* GetLandmarkObservation(Landmark* lm) {
    if (landmarks_.empty()) //SfM
    {
      return &landmark_observations_.at(lm);
    }
    else
    {
      return &keypoints_.at(lm->GetObservationIdInShot(this));
    }
  }

  void SetPose(const Pose& pose) { pose_ = pose; }
  const Pose& GetPose() const { return pose_; }
  Mat4d GetWorldToCam() const { return pose_.WorldToCamera(); }
  Mat4d GetCamToWorld() const { return pose_.CameraToWorld(); }

  void InitAndTakeDatastructures(AlignedVector<Observation> keypts,
                                 DescriptorMatrix descriptors);
  void InitKeyptsAndDescriptors(const size_t n_keypts);

  // SLAM stuff
  void UndistortedKeyptsToBearings();
  void UndistortKeypts();

  void ScalePose(const double scale);
  void ScaleLandmarks(const double scale);
  // Comparisons
  bool operator==(const Shot& shot) const { return id_ == shot.id_; }
  bool operator!=(const Shot& shot) const { return !(*this == shot); }
  bool operator<(const Shot& shot) const { return id_ < shot.id_; }
  bool operator<=(const Shot& shot) const { return id_ <= shot.id_; }
  bool operator>(const Shot& shot) const { return id_ > shot.id_; }
  bool operator>=(const Shot& shot) const { return id_ >= shot.id_; }
  // std::string GetCameraName() const { return shot_camera_.id; }
  // const Camera& GetCamera() const { return shot_camera_; }
  std::string GetCameraName() const { return shot_camera_->id; }
  const Camera* const GetCamera() const { return shot_camera_; }

  Vec2d Project(const Vec3d& global_pos) const;
  MatX2d ProjectMany(const MatX3d& points) const;

  Vec3d Bearing(const Vec2d& point) const;
  MatX3d BearingMany(const MatX2d& points) const;


  // void TransferCamOwnership(std::unique_ptr<Camera> cam)
  // {
  //   own_camera_ = std::move(cam);
  // }
 public:
  SLAMShotData slam_data_;
  const ShotId id_;  // the file name
  ShotUniqueId unique_id_;
  // const Camera& shot_camera_;
  const Camera* const shot_camera_;
  ShotMeasurements shot_measurements_;  // metadata
  ShotMesh mesh;

 private:
  Pose pose_;
  size_t num_keypts_;
  // In SLAM, the vectors have the same size as the number of detected feature
  // points and landmarks, keypoints, descriptors, etc. are linked by their
  // index
  std::vector<Landmark*> landmarks_;
  AlignedVector<Observation> keypoints_;
  DescriptorMatrix descriptors_;
  // In OpenSfM, we use a map to reproduce a similar behaviour
  std::map<Landmark*, Observation, KeyCompare,
           Eigen::aligned_allocator<Observation>>
      landmark_observations_;
  std::unordered_map<FeatureId, Landmark*> landmark_id_;
  // Workaround for pickle that makes it possible for the shot to have camera
  // outside of the reconstruction.
  std::unique_ptr<Camera> own_camera_;
};
}  // namespace map