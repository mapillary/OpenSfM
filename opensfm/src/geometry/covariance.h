#include <foundation/types.h>
#include <geometry/camera.h>
#include <geometry/functions.h>
#include <geometry/pose.h>

namespace geometry {
namespace covariance {
using PointJacobian = Eigen::Matrix<double, 2, 3, Eigen::RowMajor>;
std::pair<PointJacobian, Vec2d> ComputeJacobianReprojectionError(
    const Camera& camera, const Pose& pose, const Vec2d& observation,
    const Vec3d& point);
std::pair<Mat3d, double> ComputePointInverseCovariance(
    const std::vector<Camera>& cameras, const std::vector<Pose>& poses,
    const std::vector<Vec2d>& observations, const Vec3d& point);
}  // namespace covariance
}  // namespace geometry
