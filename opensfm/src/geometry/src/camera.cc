#include <geometry/camera.h>

#include <iostream>

namespace geometry {
Camera::Camera(const ProjectionType& type,
               const std::vector<Camera::Parameters>& types,
               const VecXd& values)
    : type_(type), types_(types), values_(values) {}

Camera Camera::CreatePerspectiveCamera(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = ProjectionType::PERSPECTIVE;
  camera.types_ = {Camera::Parameters::K1, Camera::Parameters::K2,
                   Camera::Parameters::Focal};
  camera.values_.resize(3);
  camera.values_ << k1, k2, focal;
  return camera;
};

Camera Camera::CreateBrownCamera(double focal, double aspect_ratio,
                                 const Vec2d& principal_point,
                                 const VecXd& distortion) {
  Camera camera;
  if (distortion.size() != 5) {
    throw std::runtime_error("Invalid distortion coefficients size");
  }
  camera.type_ = ProjectionType::BROWN;
  camera.types_ = {Camera::Parameters::K1,          Camera::Parameters::K2,
                   Camera::Parameters::K3,          Camera::Parameters::P1,
                   Camera::Parameters::P2,          Camera::Parameters::Focal,
                   Camera::Parameters::AspectRatio, Camera::Parameters::Cx,
                   Camera::Parameters::Cy};
  camera.values_.resize(9);
  camera.values_ << distortion[0], distortion[1], distortion[2], distortion[3],
      distortion[4], focal, aspect_ratio, principal_point[0],
      principal_point[1];
  return camera;
};

Camera Camera::CreateFisheyeCamera(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = ProjectionType::FISHEYE;
  camera.types_ = {Camera::Parameters::K1, Camera::Parameters::K2,
                   Camera::Parameters::Focal};
  camera.values_.resize(3);
  camera.values_ << k1, k2, focal;
  return camera;
};

Camera Camera::CreateFisheyeOpencvCamera(double focal, double aspect_ratio,
                                         const Vec2d& principal_point,
                                         const VecXd& distortion) {
  Camera camera;
  if (distortion.size() != 4) {
    throw std::runtime_error("Invalid distortion coefficients size");
  }
  camera.type_ = ProjectionType::FISHEYE_OPENCV;
  camera.types_ = {Camera::Parameters::K1,    Camera::Parameters::K2,
                   Camera::Parameters::K3,    Camera::Parameters::K4,
                   Camera::Parameters::Focal, Camera::Parameters::AspectRatio,
                   Camera::Parameters::Cx,    Camera::Parameters::Cy};
  camera.values_.resize(8);
  camera.values_ << distortion[0], distortion[1], distortion[2], distortion[3],
      focal, aspect_ratio, principal_point[0], principal_point[1];
  return camera;
};

/**
  Create a Fisheye62 camera with 11 parameters:
  - params: f, cx, cy, (radial) k1, k2, k3, k4, k5, k6 (tangential) p1, p2
  Note that in arvr, the parameters start at 0 and p1/p2 are reversed
  See: https://fburl.com/diffusion/xnhraa2z
*/
Camera Camera::CreateFisheye62Camera(double focal, double aspect_ratio,
                                     const Vec2d& principal_point,
                                     const VecXd& distortion) {
  if (distortion.size() != 8) {
    throw std::runtime_error("Invalid distortion coefficients size");
  }
  Camera camera;
  camera.type_ = ProjectionType::FISHEYE62;
  camera.types_ = {Camera::Parameters::K1,    Camera::Parameters::K2,
                   Camera::Parameters::K3,    Camera::Parameters::K4,
                   Camera::Parameters::K5,    Camera::Parameters::K6,
                   Camera::Parameters::P1,    Camera::Parameters::P2,
                   Camera::Parameters::Focal, Camera::Parameters::AspectRatio,
                   Camera::Parameters::Cx,    Camera::Parameters::Cy};
  camera.values_.resize(12);
  camera.values_ << distortion[0], distortion[1], distortion[2], distortion[3],
      distortion[4], distortion[5], distortion[6], distortion[7], focal,
      aspect_ratio, principal_point[0], principal_point[1];
  return camera;
}

Camera Camera::CreateDualCamera(double transition, double focal, double k1,
                                double k2) {
  Camera camera;
  camera.type_ = ProjectionType::DUAL;
  camera.types_ = {Camera::Parameters::Transition, Camera::Parameters::K1,
                   Camera::Parameters::K2, Camera::Parameters::Focal};
  camera.values_.resize(4);
  camera.values_ << transition, k1, k2, focal;
  return camera;
};

Camera Camera::CreateSphericalCamera() {
  Camera camera;
  camera.type_ = ProjectionType::SPHERICAL;
  camera.types_ = {Camera::Parameters::None};
  camera.values_.resize(1);
  camera.values_ << 0.0;
  return camera;
};

Camera Camera::CreateRadialCamera(double focal, double aspect_ratio,
                                  const Vec2d& principal_point,
                                  const Vec2d& distortion) {
  Camera camera;
  camera.type_ = ProjectionType::RADIAL;
  camera.types_ = {Camera::Parameters::K1,    Camera::Parameters::K2,
                   Camera::Parameters::Focal, Camera::Parameters::AspectRatio,
                   Camera::Parameters::Cx,    Camera::Parameters::Cy};
  camera.values_.resize(camera.types_.size());
  camera.values_ << distortion[0], distortion[1], focal, aspect_ratio,
      principal_point[0], principal_point[1];
  return camera;
}

Camera Camera::CreateSimpleRadialCamera(double focal, double aspect_ratio,
                                        const Vec2d& principal_point,
                                        double k1) {
  Camera camera;
  camera.type_ = ProjectionType::SIMPLE_RADIAL;
  camera.types_ = {Camera::Parameters::K1, Camera::Parameters::Focal,
                   Camera::Parameters::AspectRatio, Camera::Parameters::Cx,
                   Camera::Parameters::Cy};
  camera.values_.resize(camera.types_.size());
  camera.values_ << k1, focal, aspect_ratio, principal_point[0],
      principal_point[1];
  return camera;
}

std::vector<Camera::Parameters> Camera::GetParametersTypes() const {
  return types_;
}

VecXd Camera::GetParametersValues() const { return values_; }

void Camera::SetParametersValues(const VecXd& values) { values_ = values; }

std::map<Camera::Parameters, double, Camera::CompParameters>
Camera::GetParametersMap() const {
  std::map<Camera::Parameters, double, Camera::CompParameters> params_map;
  for (int i = 0; i < values_.size(); ++i) {
    params_map[types_[i]] = values_[i];
  }
  return params_map;
}

void Camera::SetParameterValue(const Parameters& parameter, double value) {
  for (int i = 0; i < values_.size(); ++i) {
    const auto type = types_[i];
    if (type == parameter) {
      values_[i] = value;
      return;
    }
  }
  throw std::runtime_error("Unknown parameter for this camera model");
}

double Camera::GetParameterValue(const Parameters& parameter) const {
  for (int i = 0; i < values_.size(); ++i) {
    const auto type = types_[i];
    if (type == parameter) {
      return values_[i];
    }
  }
  throw std::runtime_error("Unknown parameter for this camera model");
}

ProjectionType Camera::GetProjectionType() const { return type_; }

std::string Camera::GetProjectionString() const {
  return Camera::GetProjectionString(type_);
}

std::string Camera::GetProjectionString(const ProjectionType& type) {
  switch (type) {
    case ProjectionType::PERSPECTIVE:
      return "perspective";
    case ProjectionType::BROWN:
      return "brown";
    case ProjectionType::FISHEYE:
      return "fisheye";
    case ProjectionType::FISHEYE_OPENCV:
      return "fisheye_opencv";
    case ProjectionType::FISHEYE62:
      return "fisheye62";
    case ProjectionType::DUAL:
      return "dual";
    case ProjectionType::SPHERICAL:
      return "spherical";
    case ProjectionType::RADIAL:
      return "radial";
    case ProjectionType::SIMPLE_RADIAL:
      return "simple_radial";
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
}

Mat3d Camera::GetProjectionMatrix() const {
  Mat3d unnormalized = Mat3d::Zero();

  const auto params_map = GetParametersMap();

  double focal = 1.0;
  const auto find_focal = params_map.find(Camera::Parameters::Focal);
  if (find_focal != params_map.end()) {
    focal = find_focal->second;
  }

  double aspect_ratio = 1.0;
  const auto find_aspect_ratio =
      params_map.find(Camera::Parameters::AspectRatio);
  if (find_aspect_ratio != params_map.end()) {
    aspect_ratio = find_aspect_ratio->second;
  }

  Vec2d principal_point = Vec2d::Zero();
  const auto find_principal_point_x = params_map.find(Camera::Parameters::Cx);
  if (find_principal_point_x != params_map.end()) {
    principal_point(0) = find_principal_point_x->second;
  }
  const auto find_principal_point_y = params_map.find(Camera::Parameters::Cy);
  if (find_principal_point_y != params_map.end()) {
    principal_point(1) = find_principal_point_y->second;
  }

  unnormalized(0, 0) = focal;
  unnormalized(1, 1) = focal * aspect_ratio;
  unnormalized.col(2) << principal_point, 1.0;
  return unnormalized;
}

Mat3d Camera::GetProjectionMatrixScaled(int width, int height) const {
  const auto unnormalizer = std::max(width, height);
  Mat3d unnormalized = Mat3d::Zero();

  const auto projection_matrix = GetProjectionMatrix();
  unnormalized.block<2, 2>(0, 0)
      << unnormalizer * projection_matrix.block<2, 2>(0, 0);
  unnormalized.col(2) << projection_matrix(0, 2) * unnormalizer + 0.5 * width,
      projection_matrix(1, 2) * unnormalizer + 0.5 * height, 1.0;
  return unnormalized;
}

Vec2d Camera::Project(const Vec3d& point) const {
  Vec2d projected;
  Dispatch<ProjectFunction>(type_, point.data(), values_.data(),
                            projected.data());
  return projected;
}

MatX2d Camera::ProjectMany(const MatX3d& points) const {
  MatX2d projected(points.rows(), 2);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Project(points.row(i));
  }
  return projected;
}

Vec3d Camera::Bearing(const Vec2d& point) const {
  Vec3d bearing;
  Dispatch<BearingFunction>(type_, point.data(), values_.data(),
                            bearing.data());
  return bearing;
}

MatX3d Camera::BearingsMany(const MatX2d& points) const {
  MatX3d projected(points.rows(), 3);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Bearing(points.row(i));
  }
  return projected;
}

std::pair<MatXf, MatXf> ComputeCameraMapping(const Camera& from,
                                             const Camera& to, int width,
                                             int height) {
  const auto normalizer_factor = std::max(width, height);
  const auto inv_normalizer_factor = 1.0 / normalizer_factor;

  MatXf u_from(height, width);
  MatXf v_from(height, width);

  const auto half_width = width * 0.5;
  const auto half_height = height * 0.5;

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const auto uv = Vec2d(u - half_width, v - half_height);
      const Vec2d point_uv_from =
          normalizer_factor *
          from.Project(to.Bearing(inv_normalizer_factor * uv));
      u_from(v, u) = point_uv_from(0) + half_width;
      v_from(v, u) = point_uv_from(1) + half_height;
    }
  }
  return std::make_pair(u_from, v_from);
}

Vec2d Camera::PixelToNormalizedCoordinates(const Vec2d& px_coord) const {
  return PixelToNormalizedCoordinates(px_coord, width, height);
}

Vec2d Camera::NormalizedToPixelCoordinates(const Vec2d& norm_coord) const {
  return NormalizedToPixelCoordinates(norm_coord, width, height);
}

Vec2d Camera::NormalizedToPixelCoordinates(const Vec2d& norm_coord,
                                           const int width, const int height) {
  const auto size = std::max(width, height);
  return Vec2d(norm_coord[0] * size - 0.5 + width / 2.0,
               norm_coord[1] * size - 0.5 + height / 2.0);
}

Vec2d Camera::PixelToNormalizedCoordinates(const Vec2d& px_coord,
                                           const int width, const int height) {
  const auto inv_size = 1.0 / std::max(width, height);
  return Vec2d((px_coord[0] + 0.5 - width / 2.0) * inv_size,
               (px_coord[1] + 0.5 - height / 2.0) * inv_size);
}
}  // namespace geometry
