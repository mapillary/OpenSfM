#include <geometry/relative_pose.h>
#include <geometry/transform.h>

namespace geometry {
Eigen::Matrix<double, 3, 4> RelativePoseFromEssential(
    const Eigen::Matrix3d &essential, const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return ::RelativePoseFromEssential(essential, samples.begin(), samples.end());
}

Eigen::Matrix3d RelativeRotationNPoints(
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return ::RotationBetweenPoints(samples.begin(), samples.end()).transpose();
}

Eigen::Matrix<double, 3, 4> RelativePoseRefinement(
    const Eigen::Matrix<double, 3, 4> &relative_pose,
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2, int iterations) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i).normalized();
    samples[i].second = x2.row(i).normalized();
  }
  return ::RelativePoseRefinement(relative_pose, samples.begin(), samples.end(),
                                  iterations);
}
}  // namespace geometry
