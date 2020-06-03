#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace map
{

class Pose 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose():
    cam_to_world_(Mat4d::Identity()), world_to_cam_(Mat4d::Identity())
  {

  }
  // Transformation Matrices
  Mat4d WorldToCamera() const { return world_to_cam_; }
  Mat34d WorldToCameraRt() const { return world_to_cam_.block<3,4>(0,0); }

  Mat4d CameraToWorld() const { return cam_to_world_; }
  Mat34d CameraToWorldRt() const { return cam_to_world_.block<3,4>(0,0); }

  // 3x3 Rotation  
  Mat3d RotationWorldToCamera() const { return world_to_cam_.block<3,3>(0,0); }
  Mat3d RotationCameraToWorld() const { return cam_to_world_.block<3,3>(0,0); }
  Vec3d RotationWorldToCameraMin() const { return r_min_world_to_cam_;  } 
  Vec3d RotationCameraToWorldMin() const { return r_min_cam_to_world_;  } 

  // 3x1 Translation
  Vec3d TranslationWorldToCamera() const { return world_to_cam_.block<3,1>(0,3); }
  Vec3d TranslationCameraToWorld() const { return cam_to_world_.block<3,1>(0,3); };
  Vec3d GetOrigin() const { return TranslationCameraToWorld(); }  

  void SetFromWorldToCamera(const Mat4d& world_to_camera)
  {
    const Mat3d R_cw = world_to_camera.block<3,3>(0,0); //avoid ambiguous compile error
    SetFromWorldToCamera(R_cw, world_to_camera.block<3,1>(0,3));

  }
  void SetFromCameraToWorld(const Mat4d& camera_to_world)
  {
    const Mat3d R_wc = camera_to_world.block<3,3>(0,0); //avoid ambiguous compile error
    SetFromCameraToWorld(R_wc, camera_to_world.block<3,1>(0,3));
  }

  void SetFromWorldToCamera(const Mat3d& R_cw, const Vec3d& t_cw)
  {
    world_to_cam_.block<3,3>(0,0) = R_cw;
    world_to_cam_.block<3,1>(0,3) = t_cw;
    cam_to_world_ = world_to_cam_.inverse();
    UpdateMinRotations();
  }

  void SetFromCameraToWorld(const Mat3d& R_wc, const Vec3d& t_wc)
  {
    cam_to_world_.block<3,3>(0,0) = R_wc;
    cam_to_world_.block<3,1>(0,3) = t_wc;
    world_to_cam_ = cam_to_world_.inverse();
    UpdateMinRotations();
  }

  void SetWorldToCamRotation(const Vec3d& r_cw)
  {
    Mat3d R_cw = VectorToRotationMatrix(r_cw);
    world_to_cam_.block<3,3>(0,0) = R_cw;
    cam_to_world_.block<3,3>(0,0) = R_cw.transpose();
    UpdateMinRotations();
  }

  void SetWorldToCamTranslation(const Vec3d& t_cw)
  {
    world_to_cam_.block<3,1>(0,3) = t_cw;
    cam_to_world_.block<3,1>(0,3) = world_to_cam_.inverse().block<3,1>(0,3);
  }

  void SetWorldToCamRotationMatrix(const Mat3d& R_cw)
  {
    world_to_cam_.block<3,3>(0,0)= R_cw;
    cam_to_world_.block<3,3>(0,0)= R_cw.transpose();
    UpdateMinRotations();
  }
  void SetFromCameraToWorld(const Vec3d& r_wc, const Vec3d& t_wc)
  {
    const Mat3d R_wc = VectorToRotationMatrix(r_wc);
    SetFromCameraToWorld(R_wc, t_wc);

  }
  void SetFromWorldToCamera(const Vec3d& r_cw, const Vec3d& t_cw)
  {
    const Mat3d R_cw = VectorToRotationMatrix(r_cw);
    SetFromWorldToCamera(R_cw, t_cw);
  }

  Vec3d TransformWorldToCamera(const Vec3d& point) const
  {
    return world_to_cam_.block<3, 3>(0, 0) * point +
           world_to_cam_.block<3, 1>(0, 3);
  }

  Vec3d TransformCameraToWorld(const Vec3d& point) const
  {
    return cam_to_world_.block<3, 3>(0, 0) * point +
           cam_to_world_.block<3, 1>(0, 3);
  }

  Pose RelativeTo(const Pose &base_pose) const {
    Pose relpose;
    relpose.SetFromWorldToCamera(world_to_cam_ * base_pose.cam_to_world_);
    return relpose;
  }

private:
  Mat4d cam_to_world_; // [R',-R't] cam to world
  Mat4d world_to_cam_; // [R, t] world to cam
  Vec3d r_min_cam_to_world_;
  Vec3d r_min_world_to_cam_;

  static Mat3d VectorToRotationMatrix(const Vec3d& r)
  {
    const auto n = r.norm();
    if (n == 0) // avoid division by 0
    {
      return Eigen::AngleAxisd(0, r).toRotationMatrix();
    }
    else
    {
      return Eigen::AngleAxisd(n, r/n).toRotationMatrix();
    }
  }
  static Vec3d RotationMatrixToVector(const Mat3d& R)
  {
    Eigen::AngleAxisd tmp(R);
    return tmp.axis()*tmp.angle();
  }
  void UpdateMinRotations()
  {
    r_min_cam_to_world_ = RotationMatrixToVector(cam_to_world_.block<3,3>(0,0));
    r_min_world_to_cam_ = -r_min_cam_to_world_;
  }
};
}; //namespace map