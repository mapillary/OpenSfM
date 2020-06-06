#pragma once

#include <geometry/camera_functions.h>
#include <Eigen/Eigen>

class Camera {
 public:
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

  ProjectionType GetProjectionType() const;
  std::string GetProjectionString() const;

  Eigen::Matrix3d GetProjectionMatrix() const;
  Eigen::Matrix3d GetProjectionMatrixScaled(int width, int height) const;
  Eigen::Matrix3d GetProjectionMatrixScaled() const;

  bool CheckWithinBoundaries(const Eigen::Vector2d& pt) const;
  Eigen::Vector2d NormalizeImageCoordinate(const Eigen::Vector2d& pt) const;
  Eigen::Vector3d NormalizeImageCoordinateAndScale(
      const Eigen::Vector3d& pt_scale) const;
  Eigen::Vector2d UndistortImageCoordinates(const Eigen::Vector2d& pt) const;
  Eigen::Matrix<double, Eigen::Dynamic, 2> UndistortImageCoordinatesMany(const Eigen::Matrix<double, Eigen::Dynamic, 2>& pts) const;
  int width{1};
  int height{1};
  std::string id;

 private:
  Camera();

  ProjectionType type_;
  Eigen::VectorXd projection_;      // dual transition (1 = perspective, 0 = fisheye)
  Eigen::Matrix2d affine_;          // fx, skew, skew, fy = (ar*fx)
  Eigen::Vector2d principal_point_; // cx, cy
  Eigen::VectorXd distortion_;      // r^2, r^4, r^6, p1, p2
};

std::pair<Eigen::MatrixXf, Eigen::MatrixXf> ComputeCameraMapping(const Camera& from, const Camera& to, int width, int height);