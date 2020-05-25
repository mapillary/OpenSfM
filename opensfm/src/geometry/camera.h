#pragma once

#include <geometry/camera_functions.h>
#include <Eigen/Eigen>

class Camera {
 public:
  Camera();
  static Camera CreatePerspectiveCamera(double focal, double k1, double k2);
  static Camera CreateBrownCamera(double focal, double aspect_ratio,
                                  const Eigen::Vector2d& principal_point,
                                  const Eigen::VectorXd& distortion);
  static Camera CreateFisheyeCamera(double focal, double k1, double k2);
  static Camera CreateDualCamera(double transition, double focal, double k1, double k2);
  static Camera CreateSphericalCamera();

  Eigen::Vector2d Project(const Eigen::Vector3d& point) const;
  Eigen::MatrixX2d ProjectMany(const Eigen::MatrixX3d& points) const;

  Eigen::Vector3d Bearing(const Eigen::Vector2d& point) const;
  Eigen::MatrixX3d BearingsMany(const Eigen::MatrixX2d& points) const;

  void SetProjectionParams(const Eigen::VectorXd& projection);
  const Eigen::VectorXd& GetProjectionParams() const;

  void SetDistortion(const Eigen::VectorXd& distortion);
  const Eigen::VectorXd& GetDistortion() const;

  void SetPrincipalPoint(const Eigen::Vector2d& principal_point);
  Eigen::Vector2d GetPrincipalPoint() const;

  void SetFocal(double focal);
  double GetFocal() const;

  void SetAspectRatio(double focal);
  double GetAspectRatio() const;

  ProjectionType GetProjectionType()const;
  std::string GetProjectionString()const;

  Eigen::Matrix3d GetProjectionMatrix()const;
  Eigen::Matrix3d GetProjectionMatrixScaled(int width, int height)const;
  
  int width{1};
  int height{1};
  std::string id;

 private:

  ProjectionType type_;
  Eigen::VectorXd projection_;      // dual transition (1 = perspective, 0 = fisheye)
  Eigen::Matrix2d affine_;          // fx, skew, skew, fy = (ar*fx)
  Eigen::Vector2d principal_point_; // cx, cy
  Eigen::VectorXd distortion_;      // r^2, r^4, r^6, p1, p2
};

std::pair<Eigen::MatrixXf, Eigen::MatrixXf> ComputeCameraMapping(const Camera& from, const Camera& to, int width, int height);