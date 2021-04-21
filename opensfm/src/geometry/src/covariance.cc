#include <geometry/covariance.h>

namespace geometry {
namespace covariance {
using PointJacobian = Eigen::Matrix<double, 2, 3, Eigen::RowMajor>;
std::pair<PointJacobian, Vec2d> ComputeJacobianReprojectionError(
    const Camera& camera, const Pose& pose, const Vec2d& observation,
    const Vec3d& point) {
  const auto camera_parameters = camera.GetParametersValues();
  VecXd all_params(6 + camera_parameters.size());
  all_params.segment<3>(0) = pose.RotationCameraToWorldMin();
  all_params.segment<3>(3) = pose.TranslationCameraToWorld();
  for (int i = 0; i < camera_parameters.size(); ++i) {
    all_params(6 + i) = camera_parameters(i);
  }

  Vec2d projected;
  PointJacobian jacobian;
  geometry::Dispatch<geometry::ProjectPosePointDerivatives>(
      camera.GetProjectionType(), point.data(), all_params.data(),
      projected.data(), jacobian.data());
  return std::make_pair(jacobian, observation - projected);
}

std::pair<Mat3d, double> ComputePointInverseCovariance(
    const std::vector<Camera>& cameras, const std::vector<Pose>& poses,
    const std::vector<Vec2d>& observations, const Vec3d& point) {
  double sigma2 = 0;  // Assume centered errors
  Mat3d covariance = Mat3d::Zero();
  for (int i = 0; i < cameras.size(); ++i) {
    const auto result = ComputeJacobianReprojectionError(
        cameras[i], poses[i], observations[i], point);
    const auto& jacobian = result.first;
    const auto& residual = result.second;
    covariance += jacobian.transpose() * jacobian;
    sigma2 += residual.squaredNorm();
  }
  sigma2 /= cameras.size();
  return std::make_pair(covariance, sigma2);
}

}  // namespace covariance
}  // namespace geometry
