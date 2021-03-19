#pragma once

#include <foundation/types.h>
#include <geometry/camera_instances.h>

#include <unordered_map>

namespace geometry {
class Camera {
 public:
  enum class Parameters : int {
    Transition,
    K1,
    K2,
    K3,
    K4,
    K5,
    K6,
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

  Camera() = default;
  Camera(const ProjectionType& type, const std::vector<Parameters>& types,
         const VecXd& values);
  static Camera CreatePerspectiveCamera(double focal, double k1, double k2);
  static Camera CreateBrownCamera(double focal, double aspect_ratio,
                                  const Vec2d& principal_point,
                                  const VecXd& distortion);
  static Camera CreateFisheyeCamera(double focal, double k1, double k2);
  static Camera CreateFisheyeOpencvCamera(double focal, double aspect_ratio,
                                          const Vec2d& principal_point,
                                          const VecXd& distortion);
  static Camera CreateFisheye62Camera(double focal, double aspect_ratio,
                                      const Vec2d& principal_point,
                                      const VecXd& distortion);
  static Camera CreateDualCamera(double transition, double focal, double k1,
                                 double k2);
  static Camera CreateSphericalCamera();
  static Camera CreateRadialCamera(double focal, double aspect_ratio,
                                   const Vec2d& principal_point,
                                   const Vec2d& distortion);
  static Camera CreateSimpleRadialCamera(double focal, double aspect_ratio,
                                         const Vec2d& principal_point,
                                         double k1);

  Vec2d Project(const Vec3d& point) const;
  MatX2d ProjectMany(const MatX3d& points) const;

  Vec3d Bearing(const Vec2d& point) const;
  MatX3d BearingsMany(const MatX2d& points) const;

  std::vector<Parameters> GetParametersTypes() const;
  VecXd GetParametersValues() const;
  void SetParametersValues(const VecXd& values);
  std::map<Parameters, double, CompParameters> GetParametersMap() const;

  double GetParameterValue(const Parameters& parameter) const;
  void SetParameterValue(const Parameters& parameter, double value);

  ProjectionType GetProjectionType() const;
  std::string GetProjectionString() const;
  static std::string GetProjectionString(const ProjectionType& type);

  Mat3d GetProjectionMatrix() const;
  Mat3d GetProjectionMatrixScaled(int width, int height) const;

  int width{1};
  int height{1};
  std::string id;

  Vec2d PixelToNormalizedCoordinates(const Vec2d& px_coord) const;
  Vec2d NormalizedToPixelCoordinates(const Vec2d& norm_coord) const;
  static Vec2d NormalizedToPixelCoordinates(const Vec2d& norm_coord,
                                            const int width, const int height);
  static Vec2d PixelToNormalizedCoordinates(const Vec2d& px_coord,
                                            const int width, const int height);

 private:
  ProjectionType type_{ProjectionType::NONE};
  std::vector<Parameters> types_;
  VecXd values_;
};

std::pair<MatXf, MatXf> ComputeCameraMapping(const Camera& from,
                                             const Camera& to, int width,
                                             int height);
};  // namespace geometry
