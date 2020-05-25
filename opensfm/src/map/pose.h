#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <third_party/sophus/se3.hpp>

namespace map
{

class Pose 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose():
    cam_to_world_(Eigen::Matrix4d::Identity()), world_to_cam_(Eigen::Matrix4d::Identity())
  {

  }
  Eigen::Matrix4d WorldToCamera() const { return world_to_cam_.matrix(); }
  Eigen::Matrix<double, 3, 4> WorldToCameraRt() const { return world_to_cam_.matrix3x4(); }

  //4x4 Transformation
  Eigen::Matrix4d CameraToWorld() const { return cam_to_world_.matrix(); }
  Eigen::Matrix<double, 3, 4> CameraToWorldRt() const { return cam_to_world_.matrix3x4(); }

  // 3x3 Rotation
  Eigen::Matrix3d RotationWorldToCamera() const { return world_to_cam_.rotationMatrix(); }
  Eigen::Matrix3d RotationCameraToWorld() const { return cam_to_world_.rotationMatrix(); }
  Eigen::Vector3d RotationWorldToCameraMin() const { return world_to_cam_.so3().log(); }
  Eigen::Vector3d RotationCameraToWorldMin() const { return cam_to_world_.so3().log(); }

  // 3x1 Translation
  Eigen::Vector3d TranslationWorldToCamera() const { return world_to_cam_.translation(); }
  Eigen::Vector3d TranslationCameraToWorld() const { return cam_to_world_.translation(); };
  Eigen::Vector3d GetOrigin() const { return TranslationCameraToWorld(); }  

  void SetFromWorldToCamera(const Eigen::Matrix4d& world_to_camera)
  {
    const Eigen::Matrix3d R_cw = world_to_camera.block<3,3>(0,0);
    SetFromWorldToCamera(R_cw, world_to_camera.block<3,1>(0,3));

  }
  void SetFromCameraToWorld(const Eigen::Matrix4d& camera_to_world)
  {
    const Eigen::Matrix3d R_wc = camera_to_world.block<3,3>(0,0); //avoid ambiguous compile error
    SetFromCameraToWorld(R_wc, camera_to_world.block<3,1>(0,3));

    // SetFromCameraToWorld(camera_to_world.block<3,3>(0,0), camera_to_world.block<3,1>(0,3));
  }

  void SetFromWorldToCamera(const Eigen::Matrix3d& R_cw, const Eigen::Vector3d& t_cw)
  {
    world_to_cam_.setRotationMatrix(R_cw);
    world_to_cam_.translation() = t_cw;
    cam_to_world_ = world_to_cam_.inverse();
  }
  void SetFromCameraToWorld(const Eigen::Matrix3d& R_wc, const Eigen::Vector3d& t_wc)
  {
    cam_to_world_.setRotationMatrix(R_wc);
    cam_to_world_.translation() = t_wc;
    world_to_cam_ = cam_to_world_.inverse();
  }

  void SetWorldToCamRotation(const Eigen::Vector3d& r_cw)
  {
    Eigen::Matrix3d R_cw = Sophus::SO3d::exp(r_cw).matrix();
    world_to_cam_.setRotationMatrix(R_cw);
    cam_to_world_.setRotationMatrix(R_cw.transpose());
  }

  void SetWorldToCamTranslation(const Eigen::Vector3d& t_cw)
  {
    world_to_cam_.translation() = t_cw;
    cam_to_world_.translation() = world_to_cam_.inverse().translation();
  }

  void SetWorldToCamRotationMatrix(const Eigen::Matrix3d& R_cw)
  {
    world_to_cam_.setRotationMatrix(R_cw);
    cam_to_world_.setRotationMatrix(R_cw.transpose());
  }
  void SetFromCameraToWorld(const Eigen::Vector3d& R_wc, const Eigen::Vector3d& t_wc)
  {
    // Sophus::SO3d::exp(R_wc);
    // SetFromCameraToWorld(Sophus::makeRotationMatrix(R_wc), t_wc);
    SetFromCameraToWorld(Sophus::SO3d::exp(R_wc).matrix(), t_wc);

  }
  void SetFromWorldToCamera(const Eigen::Vector3d& R_cw, const Eigen::Vector3d& t_cw)
  {
    SetFromWorldToCamera(Sophus::SO3d::exp(R_cw).matrix(), t_cw);
  }

  Eigen::Vector3d TransformWorldToCamera(const Eigen::Vector3d& global_pos) const
  {
    return world_to_cam_.rotationMatrix()*global_pos + world_to_cam_.translation();
  }

  Eigen::Vector3d TransformCameraToWorld(const Eigen::Vector3d& point) const
  {
    // equal to: world_to_cam_.rotationMatrix().t (point - world_to_cam_.translation())
    return cam_to_world_.rotationMatrix()*point + cam_to_world_.translation();
  }

  // TODO: set from min representation!
private:
  Sophus::SE3d cam_to_world_;
  Sophus::SE3d world_to_cam_;
};
}; //namespace map