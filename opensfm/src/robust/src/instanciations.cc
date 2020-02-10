#include "instanciations.h"

namespace csfm {
ScoreInfo<Line::MODEL> RANSACLine(const Eigen::Matrix<double, -1, 2>& points, double threshold,
                                  const RobustEstimatorParams& parameters,
                                  const RansacType& ransac_type) {
  std::vector<Line::DATA> samples(points.rows());
  for (int i = 0; i < points.rows(); ++i) {
    samples[i] = points.row(i);
  }
  return RunEstimation<Line>(samples, threshold, parameters, ransac_type);
}

using EssentialMatrixModel = EssentialMatrix<EpipolarGeodesic>;
ScoreInfo<EssentialMatrixModel::MODEL> RANSACEssential(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters,
    const RansacType& ransac_type) {
  if((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())){
    throw std::runtime_error("Features matrices have different sizes.");
  }
  
  std::vector<EssentialMatrixModel::DATA> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return RunEstimation<EssentialMatrixModel>(samples, threshold, parameters, ransac_type);
}

using RelativePoseModel = RelativePose<EssentialMatrixSolvingFivePoints>;
ScoreInfo<Eigen::Matrix<double, 3, 4>> RANSACRelativePose(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, 
    double threshold, const RobustEstimatorParams& parameters,
    const RansacType& ransac_type) {
   if((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())){
    throw std::runtime_error("Features matrices have different sizes.");
  }
  
  std::vector<RelativePoseModel::DATA> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return RunEstimation<RelativePoseModel>(samples, threshold, parameters, ransac_type);
}

}  // namespace csfm