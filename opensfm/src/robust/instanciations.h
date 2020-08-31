#include "absolute_pose_known_rotation_model.h"
#include "absolute_pose_model.h"
#include "essential_model.h"
#include "line_model.h"
#include "relative_pose_model.h"
#include "relative_rotation_model.h"
#include "robust_estimator.h"
#include "scorer.h"

namespace robust {
ScoreInfo<Line::Type> RANSACLine(const Eigen::Matrix<double, -1, 2>& points,
                                 double threshold,
                                 const RobustEstimatorParams& parameters,
                                 const RansacType& ransac_type);

using EssentialMatrixModel = EssentialMatrix<EpipolarGeodesic>;
ScoreInfo<EssentialMatrixModel::Type> RANSACEssential(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type);

ScoreInfo<RelativePose::Type> RANSACRelativePose(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type);

ScoreInfo<RelativeRotation::Type> RANSACRelativeRotation(
    const Eigen::Matrix<double, -1, 3>& x1,
    const Eigen::Matrix<double, -1, 3>& x2, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type);

ScoreInfo<AbsolutePose::Type> RANSACAbsolutePose(
    const Eigen::Matrix<double, -1, 3>& bearings,
    const Eigen::Matrix<double, -1, 3>& points, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type);

ScoreInfo<AbsolutePoseKnownRotation::Type> RANSACAbsolutePoseKnownRotation(
    const Eigen::Matrix<double, -1, 3>& bearings,
    const Eigen::Matrix<double, -1, 3>& points, double threshold,
    const RobustEstimatorParams& parameters, const RansacType& ransac_type);
}  // namespace robust
