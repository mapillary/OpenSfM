#pragma once
#include <foundation/types.h>
#include <geometry/transformations_functions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

namespace geometry {
class Pose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose() {
    Mat4d T_cw = Mat4d::Identity();
    SetFromWorldToCamera(T_cw);
  }
  virtual ~Pose() = default;

  Pose(const Vec3d& R, const Vec3d& t = Vec3d::Zero()) {
    Mat4d T_cw = Mat4d::Identity();
    SetFromWorldToCamera(R, t);
  }
  Pose(const Mat3d& R, const Vec3d& t = Vec3d::Zero()) {
    Mat4d T_cw = Mat4d::Identity();
    T_cw.block<3, 3>(0, 0) = R;
    T_cw.block<3, 1>(0, 3) = t;
    SetFromWorldToCamera(T_cw);
  }
  // Transformation Matrices
  Mat4d WorldToCamera() const { return world_to_cam_; }
  Mat34d WorldToCameraRt() const { return world_to_cam_.block<3, 4>(0, 0); }

  Mat4d CameraToWorld() const { return cam_to_world_; }
  Mat34d CameraToWorldRt() const { return cam_to_world_.block<3, 4>(0, 0); }

  // 3x3 Rotation
  Mat3d RotationWorldToCamera() const {
    return world_to_cam_.block<3, 3>(0, 0);
  }
  Mat3d RotationCameraToWorld() const {
    return cam_to_world_.block<3, 3>(0, 0);
  }
  Vec3d RotationWorldToCameraMin() const { return r_min_world_to_cam_; }
  Vec3d RotationCameraToWorldMin() const { return r_min_cam_to_world_; }

  // 3x1 Translation
  Vec3d TranslationWorldToCamera() const {
    return world_to_cam_.block<3, 1>(0, 3);
  }

  Vec3d TranslationCameraToWorld() const {
    return cam_to_world_.block<3, 1>(0, 3);
  };

  Vec3d GetOrigin() const { return TranslationCameraToWorld(); }
  virtual void SetOrigin(const Vec3d& origin) {
    SetWorldToCamTranslation(-RotationWorldToCamera() * origin);
  }

  virtual void SetFromWorldToCamera(const Mat4d& world_to_camera) {
    const Mat3d R_cw =
        world_to_camera.block<3, 3>(0, 0);  // avoid ambiguous compile error
    SetFromWorldToCamera(R_cw, world_to_camera.block<3, 1>(0, 3));
  }
  virtual void SetFromCameraToWorld(const Mat4d& camera_to_world) {
    const Mat3d R_wc =
        camera_to_world.block<3, 3>(0, 0);  // avoid ambiguous compile error
    SetFromCameraToWorld(R_wc, camera_to_world.block<3, 1>(0, 3));
  }

  virtual void SetFromWorldToCamera(const Mat3d& R_cw, const Vec3d& t_cw) {
    world_to_cam_.setIdentity();
    world_to_cam_.block<3, 3>(0, 0) = R_cw;
    world_to_cam_.block<3, 1>(0, 3) = t_cw;
    cam_to_world_ = world_to_cam_.inverse();
    UpdateMinRotations();
  }

  virtual void SetFromCameraToWorld(const Mat3d& R_wc, const Vec3d& t_wc) {
    cam_to_world_.setIdentity();
    cam_to_world_.block<3, 3>(0, 0) = R_wc;
    cam_to_world_.block<3, 1>(0, 3) = t_wc;
    world_to_cam_ = cam_to_world_.inverse();
    UpdateMinRotations();
  }

  virtual void SetWorldToCamRotation(const Vec3d& r_cw) {
    Mat3d R_cw = geometry::VectorToRotationMatrix(r_cw);
    world_to_cam_.block<3, 3>(0, 0) = R_cw;
    cam_to_world_.block<3, 3>(0, 0) = R_cw.transpose();
    UpdateMinRotations();
  }

  virtual void SetWorldToCamTranslation(const Vec3d& t_cw) {
    world_to_cam_.block<3, 1>(0, 3) = t_cw;
    cam_to_world_.block<3, 1>(0, 3) = world_to_cam_.inverse().block<3, 1>(0, 3);
  }

  virtual void SetWorldToCamRotationMatrix(const Mat3d& R_cw) {
    world_to_cam_.block<3, 3>(0, 0) = R_cw;
    cam_to_world_.block<3, 3>(0, 0) = R_cw.transpose();
    UpdateMinRotations();
  }
  virtual void SetFromCameraToWorld(const Vec3d& r_wc, const Vec3d& t_wc) {
    const Mat3d R_wc = geometry::VectorToRotationMatrix(r_wc);
    SetFromCameraToWorld(R_wc, t_wc);
  }
  virtual void SetFromWorldToCamera(const Vec3d& r_cw, const Vec3d& t_cw) {
    const Mat3d R_cw = geometry::VectorToRotationMatrix(r_cw);
    SetFromWorldToCamera(R_cw, t_cw);
  }

  Vec3d TransformWorldToCamera(const Vec3d& point) const {
    return world_to_cam_.block<3, 3>(0, 0) * point +
           world_to_cam_.block<3, 1>(0, 3);
  }

  Vec3d TransformCameraToWorld(const Vec3d& point) const {
    return cam_to_world_.block<3, 3>(0, 0) * point +
           cam_to_world_.block<3, 1>(0, 3);
  }

  MatX3d TransformWorldToCameraMany(const MatX3d& points) const {
    const Mat3d R_cw = world_to_cam_.block<3, 3>(0, 0);
    const Vec3d t_cw = world_to_cam_.block<3, 1>(0, 3);
    return (points * R_cw.transpose()).rowwise() + t_cw.transpose();
  }

  MatX3d TransformCameraToWorldMany(const MatX3d& points) const {
    const Mat3d R_wc = cam_to_world_.block<3, 3>(0, 0);
    const Vec3d t_wc = cam_to_world_.block<3, 1>(0, 3);
    return (points * R_wc.transpose()).rowwise() + t_wc.transpose();
  }

  Pose RelativeTo(const Pose& base_pose) const {
    /*
      Computes the relative transformation between base and this post
        T_this_base = T_this_w * T_w_base
      The relation to compose is the following:
        T_pose_base = pose_CW*base_pose
        pose1.compose(pose2.inverse()) == pose1.RelativeTo(pose2)
    */
    Pose relpose;
    relpose.SetFromWorldToCamera(world_to_cam_ * base_pose.cam_to_world_);
    return relpose;
  }

  Pose Compose(const Pose& base_pose) const {
    /*
      This is the C++ version of the original Python version
      The relation to relativeTo ist the following
      pose1.compose(pose2.inverse()) == pose1.RelativeTo(pose2)
    */
    const Mat3d& selfR = RotationWorldToCamera();
    const Mat3d R = selfR * base_pose.RotationWorldToCamera();
    const Vec3d t = selfR * base_pose.TranslationWorldToCamera() +
                    TranslationWorldToCamera();
    return Pose(R, t);
  }

  Pose Inverse() const {
    /*
      Computes the inverse transformation such as :
        pose.compose(pose.inverse()) == Id
    */
    Pose invpose;
    invpose.SetFromWorldToCamera(cam_to_world_);
    return invpose;
  }

 protected:
  Mat4d cam_to_world_;  // [R',-R't] cam to world
  Mat4d world_to_cam_;  // [R, t] world to cam
  Vec3d r_min_cam_to_world_;
  Vec3d r_min_world_to_cam_;

  virtual void UpdateMinRotations() {
    r_min_cam_to_world_ =
        geometry::RotationMatrixToVector(cam_to_world_.block<3, 3>(0, 0));
    r_min_world_to_cam_ = -r_min_cam_to_world_;
  }
};

#define THROW_POSE_IMMUTABLE \
  override final { throw std::runtime_error("Cannot set an immutable pose"); }

class PoseImmutable : public geometry::Pose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit PoseImmutable(const geometry::Pose& pose) : geometry::Pose(pose) {}

  // clang-format off
  void SetOrigin(const Vec3d&) THROW_POSE_IMMUTABLE
  void SetFromWorldToCamera(const Mat4d&) THROW_POSE_IMMUTABLE
  void SetFromCameraToWorld(const Mat4d&)  THROW_POSE_IMMUTABLE
  void SetFromWorldToCamera(const Mat3d&, const Vec3d&) THROW_POSE_IMMUTABLE
  void SetFromCameraToWorld(const Mat3d&, const Vec3d&) THROW_POSE_IMMUTABLE
  void SetWorldToCamRotation(const Vec3d&) THROW_POSE_IMMUTABLE
  void SetWorldToCamTranslation(const Vec3d&)  THROW_POSE_IMMUTABLE
  void SetWorldToCamRotationMatrix(const Mat3d&)  THROW_POSE_IMMUTABLE
  void SetFromCameraToWorld(const Vec3d&, const Vec3d&)  THROW_POSE_IMMUTABLE
  void SetFromWorldToCamera(const Vec3d&, const Vec3d&)  THROW_POSE_IMMUTABLE
  // clang-format on
};
}  // namespace geometry
