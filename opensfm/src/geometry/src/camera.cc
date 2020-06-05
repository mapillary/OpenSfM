#include <geometry/camera.h>
#include <geometry/camera_functions.h>
#include <iostream>

Camera Camera::CreatePerspectiveCamera(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = ProjectionType::PERSPECTIVE;
  camera.parameters_[Camera::Parameters::Focal] = focal;
  camera.parameters_[Camera::Parameters::K1] = k1;
  camera.parameters_[Camera::Parameters::K2] = k2;
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
  camera.parameters_[Camera::Parameters::Focal] = focal;
  camera.parameters_[Camera::Parameters::AspectRatio] = aspect_ratio;
  camera.parameters_[Camera::Parameters::K1] = distortion(0);
  camera.parameters_[Camera::Parameters::K2] = distortion(1);
  camera.parameters_[Camera::Parameters::K3] = distortion(2);
  camera.parameters_[Camera::Parameters::P1] = distortion(3);
  camera.parameters_[Camera::Parameters::P2] = distortion(4);
  camera.parameters_[Camera::Parameters::Cx] = principal_point(0);
  camera.parameters_[Camera::Parameters::Cy] = principal_point(1);
};

Camera Camera::CreateFisheyeCamera(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = ProjectionType::FISHEYE;
  camera.parameters_[Camera::Parameters::Focal] = focal;
  camera.parameters_[Camera::Parameters::K1] = k1;
  camera.parameters_[Camera::Parameters::K2] = k2;
  return camera;
};

Camera Camera::CreateDualCamera(double transition, double focal, double k1,
                                double k2) {
  Camera camera;
  camera.type_ = ProjectionType::DUAL;
  camera.parameters_[Camera::Parameters::Focal] = focal;
  camera.parameters_[Camera::Parameters::K1] = k1;
  camera.parameters_[Camera::Parameters::K2] = k2;
  camera.parameters_[Camera::Parameters::Transition] = transition;
  return camera;
};

Camera Camera::CreateSphericalCamera() {
  Camera camera;
  camera.type_ = ProjectionType::SPHERICAL;
  camera.parameters_[Camera::Parameters::None] = 0.;
  return camera;
};

std::vector<Camera::Parameters> Camera::GetParametersTypes() const {
  std::vector<Camera::Parameters> types;
  for (const auto p : parameters_) {
    types.push_back(p.first);
  }
  return types;
}

VecXd Camera::GetParametersValues() const {
  VecXd values(parameters_.size());
  int count = 0;
  for (const auto p : parameters_) {
    values[count++] = p.second;
  }
  return values;
}

bool Camera::SetParameterValue(const Parameters& parameter, double value) {
  if (parameters_.find(parameter) == parameters_.end()) {
    throw std::runtime_error("Unknown parameter for this camera model");
  }
  parameters_[parameter] = value;
}

ProjectionType Camera::GetProjectionType() const { return type_; }

std::string Camera::GetProjectionString() const {
  switch (type_) {
    case ProjectionType::PERSPECTIVE:
      return "perspective";
    case ProjectionType::BROWN:
      return "brown";
    case ProjectionType::FISHEYE:
      return "fisheye";
    case ProjectionType::DUAL:
      return "dual";
    case ProjectionType::SPHERICAL:
      return "spherical";
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
}

Mat3d Camera::GetProjectionMatrix() const {
  Mat3d unnormalized = Mat3d::Zero();

  double focal = 1.0;
  const auto find_focal = parameters_.find(Camera::Parameters::Focal);
  if(find_focal != parameters_.end()){
    focal = find_focal->second;
  }

  double aspect_ratio = 1.0;
  const auto find_aspect_ratio = parameters_.find(Camera::Parameters::AspectRatio);
  if(find_aspect_ratio != parameters_.end()){
    aspect_ratio = find_aspect_ratio->second;
  }

  Vec2d principal_point = Vec2d::Zero();
  const auto find_principal_point_x = parameters_.find(Camera::Parameters::Cx);
  if(find_principal_point_x != parameters_.end()){
    principal_point(0) = find_principal_point_x->second;
  }
  const auto find_principal_point_y = parameters_.find(Camera::Parameters::Cy);
  if(find_principal_point_y != parameters_.end()){
    principal_point(1) = find_principal_point_y->second;
  }

  unnormalized(0, 0) = focal;
  unnormalized(1, 1) = focal*aspect_ratio;
  unnormalized.col(2) << principal_point, 1.0;
  return unnormalized;
}

Mat3d Camera::GetProjectionMatrixScaled(int width, int height) const {
  const auto unnormalizer = std::max(width, height);
  Mat3d unnormalized = Mat3d::Zero();

  const auto projection_matrix = GetProjectionMatrix();
  unnormalized.block<2, 2>(0, 0) << unnormalizer * unnormalized.block<2, 2>(0, 0);
  unnormalized.col(2) << projection_matrix(0, 2) * unnormalizer + 0.5 * width,
      projection_matrix(1, 2)  * unnormalizer + 0.5 * height, 1.0;
  return unnormalized;
}

Vec2d Camera::Project(const Vec3d& point) const {
  Vec2d projected;
  const auto parameters = GetParametersValues();
  Dispatch<ProjectFunction>(type_, point.data(), parameters.data(),
                            projected.data());
  return projected;
}

Eigen::MatrixX2d Camera::ProjectMany(const Eigen::MatrixX3d& points) const {
  Eigen::MatrixX2d projected(points.rows(), 2);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Project(points.row(i));
  }
  return projected;
}

Vec3d Camera::Bearing(const Vec2d& point) const {
  Vec3d bearing;
  const auto parameters = GetParametersValues();
  Dispatch<BearingFunction>(type_, point.data(), parameters.data(),
                            bearing.data());
  return bearing;
}

Eigen::MatrixX3d Camera::BearingsMany(const Eigen::MatrixX2d& points) const {
  Eigen::MatrixX3d projected(points.rows(), 3);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Bearing(points.row(i));
  }
  return projected;
}

Camera::Camera() : type_(ProjectionType::PERSPECTIVE) {
}

std::pair<MatXf, MatXf> ComputeCameraMapping(const Camera& from, const Camera& to, int width, int height){
  const auto normalizer_factor = std::max(width, height);
  const auto inv_normalizer_factor = 1.0/normalizer_factor;

  MatXf u_from(height, width);
  MatXf v_from(height, width);

  const auto half_width = width*0.5;
  const auto half_height = height*0.5;

  for(int v = 0; v < height; ++v){
    for(int u = 0; u < width; ++u){
      const auto uv = Vec2d(u-half_width, v-half_height);
      const Vec2d point_uv_from = normalizer_factor*from.Project(to.Bearing(inv_normalizer_factor*uv));
      u_from(v, u) = point_uv_from(0) + half_width;
      v_from(v, u) = point_uv_from(1) + half_height;
    }
  }
  return std::make_pair(u_from, v_from);
}
