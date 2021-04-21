#include <geometry/absolute_pose.h>

Eigen::Matrix3d RotationMatrixAroundAxis(const double cos_theta,
                                         const double sin_theta,
                                         const Eigen::Vector3d &v) {
  Eigen::Matrix3d R;
  const auto one_minus_cos_theta = 1.0 - cos_theta;
  R(0, 0) = cos_theta + v[0] * v[0] * one_minus_cos_theta;
  R(1, 0) = -v[2] * sin_theta + v[0] * v[1] * one_minus_cos_theta;
  R(2, 0) = v[1] * sin_theta + v[0] * v[2] * one_minus_cos_theta;
  R(0, 1) = v[2] * sin_theta + v[0] * v[1] * one_minus_cos_theta;
  R(1, 1) = cos_theta + v[1] * v[1] * one_minus_cos_theta;
  R(2, 1) = -v[0] * sin_theta + v[1] * v[2] * one_minus_cos_theta;
  R(0, 2) = -v[1] * sin_theta + v[0] * v[2] * one_minus_cos_theta;
  R(1, 2) = v[0] * sin_theta + v[1] * v[2] * one_minus_cos_theta;
  R(2, 2) = cos_theta + v[2] * v[2] * one_minus_cos_theta;
  return R;
}

namespace geometry {
std::vector<Eigen::Matrix<double, 3, 4>> AbsolutePoseThreePoints(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(
      bearings.rows());
  for (int i = 0; i < bearings.rows(); ++i) {
    samples[i].first = bearings.row(i).normalized();
    samples[i].second = points.row(i);
  }
  return ::AbsolutePoseThreePoints(samples.begin(), samples.end());
}

Eigen::Matrix<double, 3, 4> AbsolutePoseNPoints(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(
      bearings.rows());
  for (int i = 0; i < bearings.rows(); ++i) {
    samples[i].first = bearings.row(i).normalized();
    samples[i].second = points.row(i);
  }
  return ::AbsolutePoseNPoints(samples.begin(), samples.end());
}

Eigen::Vector3d AbsolutePoseNPointsKnownRotation(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(
      bearings.rows());
  for (int i = 0; i < bearings.rows(); ++i) {
    samples[i].first = bearings.row(i).normalized();
    samples[i].second = points.row(i);
  }
  return ::AbsolutePoseNPointsKnownRotation(samples.begin(), samples.end());
}
}  // namespace geometry
