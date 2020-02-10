#pragma once

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <fstream>
#include <iostream>
#include <string>
#include <foundation/types.h>

namespace geometry {

enum {
  TRIANGULATION_OK = 0,
  TRIANGULATION_SMALL_ANGLE,
  TRIANGULATION_BEHIND_CAMERA,
  TRIANGULATION_BAD_REPROJECTION
};

double AngleBetweenVectors(const Eigen::Vector3d &u, const Eigen::Vector3d &v);

py::list TriangulateReturn(int error, py::object value);

Eigen::Vector4d TriangulateBearingsDLTSolve(
    const Eigen::Matrix<double, 3, -1> &bs,
    const std::vector< Eigen::Matrix<double, 3, 4> > &Rts);

py::object TriangulateBearingsDLT(const py::list &Rts_list,
                                  const py::list &bs_list, double threshold,
                                  double min_angle);

// Point minimizing the squared distance to all rays
// Closed for solution from
//   Srikumar Ramalingam, Suresh K. Lodha and Peter Sturm
//   "A generic structure-from-motion framework"
//   CVIU 2006
Eigen::Vector3d TriangulateBearingsMidpointSolve(
    const Eigen::Matrix<double, 3, -1> &os,
    const Eigen::Matrix<double, 3, -1> &bs);

py::object TriangulateBearingsMidpoint(const py::list &os_list,
                                       const py::list &bs_list,
                                       const py::list &threshold_list,
                                       double min_angle);

}  // namespace geometry
