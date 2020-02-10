#include "robust_estimator.h"
#include "scorer.h"
#include "line_model.h"
#include "essential_model.h"
#include "relative_pose_model.h"

namespace csfm {
ScoreInfo<Line::MODEL> RANSACLine(const Eigen::Matrix<double, -1, 2>& points, double threshold,
                                  const RobustEstimatorParams& parameters,
                                  const RansacType& ransac_type);

using EssentialMatrixModel = EssentialMatrix<EpipolarGeodesic>;
ScoreInfo<EssentialMatrixModel::MODEL> RANSACEssential(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters,
    const RansacType& ransac_type);

using RelativePoseModel = RelativePose<EssentialMatrixSolvingFivePoints>;
ScoreInfo<Eigen::Matrix<double, 3, 4>> RANSACRelativePose(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, 
    double threshold, const RobustEstimatorParams& parameters,
    const RansacType& ransac_type);

}  // namespace csfm