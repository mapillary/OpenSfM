#include <geometry/absolute_pose.h>

std::vector<Eigen::Matrix<double, 3, 4>> AbsolutePoseThreePoints(
  const Eigen::Matrix<double, -1, 3> &bearings,
  const Eigen::Matrix<double, -1, 3> &points){
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(bearings.rows());
  for (int i = 0; i < bearings.rows(); ++i) {
    samples[i].first = bearings.row(i);
    samples[i].second = points.row(i);
  }
  return ::AbsolutePoseThreePoints(samples.begin(), samples.end());
}