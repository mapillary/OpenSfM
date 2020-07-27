#pragma once

#include <geometry/camera_functions.h>

#include <Eigen/Eigen>
#include <unordered_map>

class Camera {
 public:
  enum class Parameters : int {
    Transition,
    K1,
    K2,
    K3,
    P1,
    P2,
    Focal,
    AspectRatio,
    Cx,
    Cy,
    None
  };

  struct CompParameters {
    size_t operator()(const Parameters& p1, const Parameters& p2) const {
      return static_cast<size_t>(p1) < static_cast<size_t>(p2);
    }
  };

  Camera(const std::vector<Camera::Parameters>& types, const VecXd& values);
  static Camera CreatePerspectiveCamera(double focal, double k1, double k2);
  static Camera CreateBrownCamera(double focal, double aspect_ratio,
                                  const Vec2d& principal_point,
                                  const VecXd& distortion);
  static Camera CreateFisheyeCamera(double focal, double k1, double k2);
  static Camera CreateDualCamera(double transition, double focal, double k1,
                                 double k2);
  static Camera CreateSphericalCamera();

  Vec2d Project(const Vec3d& point) const;
  Eigen::MatrixX2d ProjectMany(const Eigen::MatrixX3d& points) const;

  Vec3d Bearing(const Vec2d& point) const;
  Eigen::MatrixX3d BearingsMany(const Eigen::MatrixX2d& points) const;

  std::vector<Parameters> GetParametersTypes() const;
  VecXd GetParametersValues() const;
  std::map<Parameters, double, CompParameters> GetParametersMap()const;

  double GetParameterValue(const Parameters& parameter) const;
  void SetParameterValue(const Parameters& parameter, double value);

  ProjectionType GetProjectionType() const;
  std::string GetProjectionString() const;

  Mat3d GetProjectionMatrix() const;
  Mat3d GetProjectionMatrixScaled(int width, int height) const;

  int width{1};
  int height{1};
  std::string id;

 private:
  Camera();

  ProjectionType type_;
  std::vector<Parameters> types_;
  VecXd values_;
};

std::pair<MatXf, MatXf> ComputeCameraMapping(const Camera& from,
                                             const Camera& to, int width,
                                             int height);