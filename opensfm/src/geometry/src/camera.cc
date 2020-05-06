#include <geometry/camera.h>

Camera Camera::CreatePerspective(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = Type::PERSPECTIVE;
  camera.affine_ << focal, 0, 0, focal;
  camera.distorsion_ << k1, k2, 0, 0, 0;
  return camera;
};

Camera Camera::CreateBrownCamera(double focal_x, double focal_y,
                                 const Eigen::Vector2d& principal_point,
                                 const Eigen::VectorXd& distorsion) {
  Camera camera;
  if (distorsion.size() != camera.distorsion_.size()) {
    throw std::runtime_error("Invalid distorsion coefficients size");
  }
  camera.type_ = Type::BROWN;
  camera.affine_ << focal_x, 0, 0, focal_y;
  camera.distorsion_ = distorsion;
  camera.principal_point_ = principal_point;
  return camera;
};

Camera Camera::CreateFisheyeCamera(double focal, double k1, double k2) {
  Camera camera;
  camera.type_ = Type::FISHEYE;
  camera.affine_ << focal, 0, 0, focal;
  camera.distorsion_ << k1, k2, 0, 0, 0;
  return camera;
};

Camera Camera::CreateSphericalCamera() {
  Camera camera;
  camera.type_ = Type::SPHERICAL;
  return camera;
};

Eigen::Vector2d Camera::Project(const Eigen::Vector3d& point) const {
  return Dispatch<Eigen::Vector2d, ProjectT, Eigen::Vector3d>(point);
}

Eigen::MatrixX2d Camera::ProjectMany(const Eigen::MatrixX3d& points) const {
  Eigen::MatrixX2d projected(points.rows(), 2);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Project(points.row(i));
  }
  return projected;
}

Eigen::Vector3d Camera::Bearing(const Eigen::Vector2d& point) const {
  return Dispatch<Eigen::Vector3d, BearingT, Eigen::Vector2d>(point);
}

Eigen::MatrixX3d Camera::BearingsMany(const Eigen::MatrixX2d& points) const {
  Eigen::MatrixX3d projected(points.rows(), 3);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Bearing(points.row(i));
  }
  return projected;
}

Camera::Camera() : type_(Type::PERSPECTIVE) {
  affine_.setIdentity();
  principal_point_.setZero();
  distorsion_.resize(5);
  distorsion_.setZero();
}

// This is where the pseudo-strategy pattern takes place.
// If you want to add your own new camera model, just add
// a new enum value and the corresponding case below.
template <class OUT, class FUNC, class... IN>
OUT Camera::Dispatch(IN... args) const {
  switch (type_) {
    case PERSPECTIVE:
      return FUNC::template Apply<PerspectiveProjection, Disto24, Affine>(
          args..., affine_, principal_point_, distorsion_);
    case BROWN:
      return FUNC::template Apply<PerspectiveProjection, DistoBrown, Affine>(
          args..., affine_, principal_point_, distorsion_);
    case FISHEYE:
      return FUNC::template Apply<FisheyeProjection, Disto24, Affine>(
          args..., affine_, principal_point_, distorsion_);
    case SPHERICAL:
      return FUNC::template Apply<SphericalProjection, Identity, Identity>(
          args..., affine_, principal_point_, distorsion_);
  }
}