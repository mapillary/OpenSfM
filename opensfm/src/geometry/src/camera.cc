#include <geometry/camera.h>
#include <geometry/camera_functions.h>
#include <iostream>

Camera Camera::CreatePerspectiveCamera(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = ProjectionType::PERSPECTIVE;
  camera.affine_ << focal, 0, 0, focal;
  camera.distortion_ << k1, k2, 0, 0, 0;
  return camera;
};

Camera Camera::CreateBrownCamera(double focal, double aspect_ratio,
                                 const Eigen::Vector2d& principal_point,
                                 const Eigen::VectorXd& distortion) {
  Camera camera;
  if (distortion.size() != camera.distortion_.size()) {
    throw std::runtime_error("Invalid distortion coefficients size");
  }
  camera.type_ = ProjectionType::BROWN;
  camera.affine_ << focal, 0, 0, focal * aspect_ratio;
  camera.distortion_ = distortion;
  camera.principal_point_ = principal_point;
  return camera;
};

Camera Camera::CreateFisheyeCamera(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = ProjectionType::FISHEYE;
  camera.affine_ << focal, 0, 0, focal;
  camera.distortion_ << k1, k2, 0, 0, 0;
  return camera;
};

Camera Camera::CreateDualCamera(double transition, double focal, double k1,
                                double k2) {
  Camera camera;
  camera.type_ = ProjectionType::DUAL;
  camera.projection_[0] = transition;
  camera.affine_ << focal, 0, 0, focal;
  camera.distortion_ << k1, k2, 0, 0, 0;
  return camera;
};

Camera Camera::CreateSphericalCamera() {
  Camera camera;
  camera.type_ = ProjectionType::SPHERICAL;
  return camera;
};

void Camera::SetProjectionParams(const Eigen::VectorXd& projection) {
  if (type_ != ProjectionType::DUAL) {
    return;
  }
  if (type_ == ProjectionType::DUAL && projection.size() > 1) {
    throw std::runtime_error("Incorrect size of projection parameters");
  }
  projection_ = projection;
}

const Eigen::VectorXd& Camera::GetProjectionParams() const {
  return projection_;
}

void Camera::SetDistortion(const Eigen::VectorXd& distortion) {
  std::vector<int> coeffs;
  if (type_ != ProjectionType::SPHERICAL) {
    coeffs.push_back(static_cast<int>(Disto::K1));
    coeffs.push_back(static_cast<int>(Disto::K2));
  }
  if (type_ == ProjectionType::BROWN) {
    coeffs.push_back(static_cast<int>(Disto::K3));
    coeffs.push_back(static_cast<int>(Disto::P1));
    coeffs.push_back(static_cast<int>(Disto::P2));
  }
  for (const auto idx : coeffs) {
    distortion_[idx] = distortion[idx];
  }
}
const Eigen::VectorXd& Camera::GetDistortion() const { return distortion_; }

void Camera::SetPrincipalPoint(const Eigen::Vector2d& principal_point) {
  if (type_ != ProjectionType::BROWN) {
    return;
  }
  principal_point_ = principal_point;
}
Eigen::Vector2d Camera::GetPrincipalPoint() const { return principal_point_; }

void Camera::SetFocal(double focal) {
  if (type_ == ProjectionType::SPHERICAL) {
    return;
  }

  const auto ar = GetAspectRatio();
  affine_(1, 1) = ar * focal;
  affine_(0, 0) = focal;
}
double Camera::GetFocal() const { return affine_(0, 0); }

void Camera::SetAspectRatio(double ar) {
  if (type_ != ProjectionType::BROWN) {
    return;
  }
  affine_(1, 1) = ar * GetFocal();
}

double Camera::GetAspectRatio() const { return affine_(1, 1) / affine_(0, 0); }

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

Eigen::Matrix3d Camera::GetProjectionMatrix() const {
  Eigen::Matrix3d unnormalized = Eigen::Matrix3d::Zero();
  unnormalized << affine_;
  unnormalized.col(2) << principal_point_, 1.0;
  return unnormalized;
}

Eigen::Matrix3d Camera::GetProjectionMatrixScaled(int width, int height) const {
  const auto unnormalizer = std::max(width, height);

  Eigen::Matrix3d unnormalized = Eigen::Matrix3d::Zero();
  unnormalized << unnormalizer * affine_;
  unnormalized.col(2) << principal_point_[0] * unnormalizer + 0.5 * width,
      principal_point_[1] * unnormalizer + 0.5 * height, 1.0;
  return unnormalized;
}

Eigen::Vector2d Camera::Project(const Eigen::Vector3d& point) const {
  return Dispatch<Eigen::Vector2d, ProjectFunction>(
      type_, point, projection_, affine_, principal_point_, distortion_);
}

Eigen::MatrixX2d Camera::ProjectMany(const Eigen::MatrixX3d& points) const {
  Eigen::MatrixX2d projected(points.rows(), 2);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Project(points.row(i));
  }
  return projected;
}

Eigen::Vector3d Camera::Bearing(const Eigen::Vector2d& point) const {
  return Dispatch<Eigen::Vector3d, BearingFunction>(
      type_, point, projection_, affine_, principal_point_, distortion_);
}

Eigen::MatrixX3d Camera::BearingsMany(const Eigen::MatrixX2d& points) const {
  Eigen::MatrixX3d projected(points.rows(), 3);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Bearing(points.row(i));
  }
  return projected;
}

Camera::Camera() : type_(ProjectionType::PERSPECTIVE) {
  projection_.resize(1);
  projection_[0] = 1.0;
  affine_.setIdentity();
  principal_point_.setZero();
  distortion_.resize(static_cast<int>(Disto::COUNT));
  distortion_.setZero();
}

std::pair<Eigen::MatrixXf, Eigen::MatrixXf> ComputeCameraMapping(const Camera& from, const Camera& to, int width, int height){
  const auto normalizer_factor = std::max(width, height);
  const auto inv_normalizer_factor = 1.0/normalizer_factor;

  Eigen::MatrixXf u_from(height, width);
  Eigen::MatrixXf v_from(height, width);

  const auto half_width = width*0.5;
  const auto half_height = height*0.5;

  for(int v = 0; v < height; ++v){
    for(int u = 0; u < width; ++u){
      const auto uv = Eigen::Vector2d(u-half_width, v-half_height);
      const Eigen::Vector2d point_uv_from = normalizer_factor*from.Project(to.Bearing(inv_normalizer_factor*uv));
      u_from(v, u) = point_uv_from(0) + half_width;
      v_from(v, u) = point_uv_from(1) + half_height;
    }
  }
  return std::make_pair(u_from, v_from);
}
