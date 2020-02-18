#pragma once

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <ceres/tiny_solver.h>
#include <ceres/rotation.h>
#include <ceres/tiny_solver_autodiff_function.h>
#include "triangulation.h"

template <class IT>
Eigen::Matrix<double, 3, 4> RelativePoseFromEssential(
    const Eigen::Matrix3d& essential, IT begin, IT end) {

  Eigen::JacobiSVD<Eigen::Matrix3d> SVD(essential, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix3d U = SVD.matrixU();
  Eigen::Matrix3d Vt = SVD.matrixV().transpose();

  // Last column of U is undetermined since d = (a a 0).
  if (U.determinant() < 0) {
    U.col(2) *= -1;
  }
  // Last row of Vt is undetermined since d = (a a 0).
  if (Vt.determinant() < 0) {
    Vt.row(2) *= -1;
  }

  Eigen::Matrix3d W;
  W << 0, -1, 0, 
       1, 0, 0, 
       0, 0, 1;

  Eigen::Matrix<double, 2, 3> bearings;
  Eigen::Matrix<double, 2, 3> centers;
  centers.row(0) = Eigen::Vector3d::Zero();

  auto best_decomposition = std::make_pair(0, Eigen::Matrix<double, 3, 4>());
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector3d translation;
    if (i == 0) {
      translation = U.col(2);
    } else {
      translation = -U.col(2);
    }
    translation.normalize();
    for (int j = 0; j < 2; ++j) {
      Eigen::Matrix3d rotation;
      if (j == 0) {
        rotation = U * W * Vt;
      } else {
        rotation = U * W.transpose() * Vt;
      }

      // Since rotation=Rcamera and translation=Tcamera parametrization
      centers.row(1) = -rotation.transpose()*translation;

      int are_in_front = 0;
      for (IT it = begin; it != end; ++it) {
        bearings.row(0) = it->first;
        bearings.row(1) = rotation.transpose()*it->second;
        const Eigen::Vector3d point = geometry::TriangulateTwoBearingsMidpointSolve(centers, bearings);
        const bool is_in_front =
            bearings.row(0).dot(point) > 0.0 &&
            bearings.row(1).dot(rotation * point + translation) > 0.0;
        are_in_front += is_in_front;

        const auto maximum_possible = are_in_front + (end - it);
        if (maximum_possible < best_decomposition.first) {
          break;
        }
      }

      if (are_in_front > best_decomposition.first) {
        Eigen::Matrix<double, 3, 4> RT;
        RT.block<3, 3>(0, 0) = rotation;
        RT.block<3, 1>(0, 3) = translation;
        best_decomposition = std::make_pair(are_in_front, RT);
      }
    }
  }

  // Rcamera and Tcamera parametrization
  return best_decomposition.second;
}

template< class IT >
struct RelativePoseCost {
  RelativePoseCost(IT begin, IT end): begin_(begin), end_(end){}
  int NumResiduals() const {
    return (end_-begin_)+1;
  }

  template<typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> rotation(parameters);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(parameters+3);
    Eigen::Matrix<T, 3, 1> rotation_transpose = -rotation;

    Eigen::Matrix<T, 2, 3> bearings;
    Eigen::Matrix<T, 2, 3> centers;
    centers.row(0) = Eigen::Matrix<T, 3, 1>::Zero();
    centers.row(1) = translation;
    Eigen::Matrix<T, 3, 1> some_tmp = Eigen::Matrix<T, 3, 1>::Zero();

    int count = 0;
    residuals[0] = T(0.);
    for (IT it = begin_; it != end_; ++it) {
      const Eigen::Matrix<T, 3, 1> x = it->first.template cast<T>();
      const Eigen::Matrix<T, 3, 1> y = it->second.template cast<T>();

      // Bearing : x stay at identity, y is brought to the world with R(t)
      bearings.row(0) = x;
      ceres::AngleAxisRotatePoint(rotation_transpose.data(), y.data(), some_tmp.data());
      bearings.row(1) = some_tmp;

      // Triangulate
      auto point = geometry::TriangulateTwoBearingsMidpointSolve(centers, bearings);

      // Point in x stays at identity, y is brought in second camera with R*(y-t)
      const auto projected_x = point.normalized();
      const Eigen::Matrix<T, 3, 1> y_centered = point-translation;
      ceres::AngleAxisRotatePoint(rotation.data(), y_centered.data(), some_tmp.data());
      const auto projected_y = some_tmp.normalized();
      residuals[0] += 1.0 - ((projected_x.dot(x) + projected_y.dot(y))*T(0.5));
    }
    residuals[1] = 1.0 - translation.norm();
  }
  IT begin_;
  IT end_;
};

template <class IT>
Eigen::Matrix<double, 3, 4> RelativePoseRefinement(
  const Eigen::Matrix<double, 3, 4>& relative_pose, IT begin, IT end, int iterations) {
  Eigen::Matrix<double, 6, 1> parameters;
  parameters.setZero();

  // Use Rcamera and Tworld parametrization
  ceres::RotationMatrixToAngleAxis(relative_pose.block<3, 3>(0, 0).data(), parameters.data());
  parameters.segment<3>(3) = -relative_pose.block<3, 3>(0, 0).transpose()*relative_pose.col(3);

  using RelativePoseFunction = ceres::TinySolverAutoDiffFunction<RelativePoseCost<IT>, 2, 6>;
  RelativePoseCost<IT> cost(begin, end);
  RelativePoseFunction f(cost);

  ceres::TinySolver<RelativePoseFunction> solver;
  solver.options.max_num_iterations = iterations;
  solver.Solve(f, &parameters);

  // Back with Tcamera parametrization
  Eigen::Matrix<double, 3, 4> relative_pose_result;
  ceres::AngleAxisToRotationMatrix(parameters.data(), relative_pose_result.block<3, 3>(0, 0).data());
  relative_pose_result.col(3) = -relative_pose_result.block<3, 3>(0, 0)*parameters.tail<3>(3);
  return relative_pose_result;
}
namespace geometry {
Eigen::Matrix<double, 3, 4> RelativePoseFromEssential(
    const Eigen::Matrix3d& essential,
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2);
Eigen::Matrix<double, 3, 4> RelativePoseRefinement(
    const Eigen::Matrix<double, 3, 4>& relative_pose,
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2,
    int iterations);
}