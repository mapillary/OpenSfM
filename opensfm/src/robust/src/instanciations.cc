#include <robust/instanciations.h>

namespace robust {
ScoreInfo<Line::Type> RANSACLine(const Eigen::Matrix<double, -1, 2>& points,
                                 double threshold,
                                 const RobustEstimatorParams& parameters,
                                 const RansacType& ransac_type) {
  std::vector<Line::Data> samples(points.rows());
  for (int i = 0; i < points.rows(); ++i) {
    samples[i] = points.row(i);
  }
  return RunEstimation<Line>(samples, threshold, parameters, ransac_type);
}

using EssentialMatrixModel = EssentialMatrix<EpipolarGeodesic>;
ScoreInfo<EssentialMatrixModel::Type> RANSACEssential(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }

  std::vector<EssentialMatrixModel::Data> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return RunEstimation<EssentialMatrixModel>(samples, threshold, parameters,
                                             ransac_type);
}

ScoreInfo<RelativePose::Type> RANSACRelativePose(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }

  std::vector<RelativePose::Data> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return RunEstimation<RelativePose>(samples, threshold, parameters,
                                     ransac_type);
}

ScoreInfo<RelativeRotation::Type> RANSACRelativeRotation(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }

  std::vector<RelativeRotation::Data> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return RunEstimation<RelativeRotation>(samples, threshold, parameters,
                                         ransac_type);
}

ScoreInfo<AbsolutePose::Type> RANSACAbsolutePose(
    const Eigen::Matrix<double, -1, 3>& bearings,
    const Eigen::Matrix<double, -1, 3>& points, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type) {
  if ((bearings.cols() != points.cols()) ||
      (bearings.rows() != points.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }

  std::vector<AbsolutePose::Data> samples(bearings.rows());
  for (int i = 0; i < bearings.rows(); ++i) {
    samples[i].first = bearings.row(i).normalized();
    samples[i].second = points.row(i);
  }
  return RunEstimation<AbsolutePose>(samples, threshold, parameters,
                                     ransac_type);
}

ScoreInfo<AbsolutePoseKnownRotation::Type> RANSACAbsolutePoseKnownRotation(
    const Eigen::Matrix<double, -1, 3>& bearings,
    const Eigen::Matrix<double, -1, 3>& points, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type) {
  if ((bearings.cols() != points.cols()) ||
      (bearings.rows() != points.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }

  std::vector<AbsolutePoseKnownRotation::Data> samples(bearings.rows());
  for (int i = 0; i < bearings.rows(); ++i) {
    samples[i].first = bearings.row(i).normalized();
    samples[i].second = points.row(i);
  }
  return RunEstimation<AbsolutePoseKnownRotation>(samples, threshold,
                                                  parameters, ransac_type);
}

}  // namespace robust
