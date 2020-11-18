
#include <geometry/triangulation.h>

double AngleBetweenVectors(const Eigen::Vector3d &u, const Eigen::Vector3d &v) {
  double c = (u.dot(v)) / sqrt(u.dot(u) * v.dot(v));
  if (std::fabs(c) >= 1.0)
    return 0.0;
  else
    return acos(c);
}

Eigen::Vector4d TriangulateBearingsDLTSolve(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const std::vector<Eigen::Matrix<double, 3, 4>> &Rts) {
  const int nviews = bearings.rows();
  assert(nviews == Rts.size());

  Eigen::MatrixXd A(2 * nviews, 4);
  for (int i = 0; i < nviews; i++) {
    A.row(2 * i) =
        bearings(i, 0) * Rts[i].row(2) - bearings(i, 2) * Rts[i].row(0);
    A.row(2 * i + 1) =
        bearings(i, 1) * Rts[i].row(2) - bearings(i, 2) * Rts[i].row(1);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> mySVD(A, Eigen::ComputeFullV);
  Eigen::Vector4d worldPoint;
  worldPoint[0] = mySVD.matrixV()(0, 3);
  worldPoint[1] = mySVD.matrixV()(1, 3);
  worldPoint[2] = mySVD.matrixV()(2, 3);
  worldPoint[3] = mySVD.matrixV()(3, 3);

  return worldPoint;
}

namespace geometry {

std::pair<bool, Eigen::Vector3d> TriangulateBearingsDLT(
    const std::vector<Eigen::Matrix<double, 3, 4>> &Rts,
    const Eigen::Matrix<double, -1, 3> &bearings, double threshold,
    double min_angle) {
  const int count = Rts.size();
  Eigen::MatrixXd world_bearings(count, 3);
  bool angle_ok = false;
  for (int i = 0; i < count && !angle_ok; ++i) {
    const Eigen::Matrix<double, 3, 4> Rt = Rts[i];
    world_bearings.row(i) =
        Rt.block<3, 3>(0, 0).transpose() * bearings.row(i).transpose();
    for (int j = 0; j < i && !angle_ok; ++j) {
      const double angle =
          AngleBetweenVectors(world_bearings.row(i), world_bearings.row(j));
      if (angle >= min_angle) {
        angle_ok = true;
      }
    }
  }

  if (!angle_ok) {
    return std::make_pair(false, Eigen::Vector3d());
  }

  Eigen::Vector4d X = TriangulateBearingsDLTSolve(bearings, Rts);
  X /= X(3);

  for (int i = 0; i < count; ++i) {
    const Eigen::Vector3d projected = Rts[i] * X;
    if (AngleBetweenVectors(projected, bearings.row(i)) > threshold) {
      return std::make_pair(false, Eigen::Vector3d());
    }
  }

  return std::make_pair(true, X.head<3>());
}

std::vector<std::pair<bool, Eigen::Vector3d>>
TriangulateTwoBearingsMidpointMany(
    const Eigen::Matrix<double, -1, 3> &bearings1,
    const Eigen::Matrix<double, -1, 3> &bearings2,
    const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) {
  std::vector<std::pair<bool, Eigen::Vector3d>> triangulated(bearings1.rows());
  Eigen::Matrix<double, 2, 3> os, bs;
  os.row(0) = Eigen::Vector3d::Zero();
  os.row(1) = translation;
  for (int i = 0; i < bearings1.rows(); ++i) {
    bs.row(0) = bearings1.row(i);
    bs.row(1) = rotation * bearings2.row(i).transpose();
    triangulated[i] = TriangulateTwoBearingsMidpointSolve(os, bs);
  }
  return triangulated;
}

std::pair<bool, Eigen::Vector3d> TriangulateBearingsMidpoint(
    const Eigen::Matrix<double, -1, 3> &centers,
    const Eigen::Matrix<double, -1, 3> &bearings,
    const std::vector<double> &threshold_list, double min_angle) {
  const int count = centers.rows();

  // Check angle between rays
  bool angle_ok = false;
  for (int i = 0; i < count && !angle_ok; ++i) {
    for (int j = 0; j < i && !angle_ok; ++j) {
      const auto angle = AngleBetweenVectors(bearings.row(i), bearings.row(j));
      if (angle >= min_angle) {
        angle_ok = true;
      }
    }
  }
  if (!angle_ok) {
    return std::make_pair(false, Eigen::Vector3d());
  }

  // Triangulate
  const auto X = TriangulateBearingsMidpointSolve(centers, bearings);

  // Check reprojection error
  for (int i = 0; i < count; ++i) {
    const Eigen::Vector3d projected = X - centers.row(i).transpose();
    const Eigen::Vector3d measured = bearings.row(i);
    if (AngleBetweenVectors(projected, measured) > threshold_list[i]) {
      return std::make_pair(false, Eigen::Vector3d());
    }
  }

  return std::make_pair(true, X.head<3>());
}

}  // namespace geometry
