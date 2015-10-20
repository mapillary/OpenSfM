
#include <iostream>
#include <fstream>
#include <string>
#include "types.h"
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/StdVector>


namespace csfm {


typedef std::vector<Eigen::Matrix<double, 3, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4> > > vector_mat34;

enum {
  TRIANGULATION_OK = 0,
  TRIANGULATION_SMALL_ANGLE,
  TRIANGULATION_BEHIND_CAMERA,
  TRIANGULATION_BAD_REPROJECTION
};


double AngleBetweenVectors(const Eigen::Vector3d &u,
                           const Eigen::Vector3d &v) {
    double c = (u.dot(v))
               / sqrt(u.dot(u) * v.dot(v));
    if (c >= 1.0) return 0.0;
    else return acos(c);
}


bp::object TriangulateReturn(int error, bp::object value) {
    bp::list retn;
    retn.append(int(error));
    retn.append(value);
    return retn;
}


Eigen::Vector4d TriangulateBearingsDLT(
    const Eigen::Matrix<double, 3, Eigen::Dynamic> &bs,
    const vector_mat34 &Rts) {
  int nviews = bs.cols();
  assert(nviews == Rts.size());

  Eigen::MatrixXd A(2 * nviews, 4);
  for (int i = 0; i < nviews; i++) {
    A.row(2 * i    ) = bs(0, i) * Rts[i].row(2) - bs(2, i) * Rts[i].row(0);
    A.row(2 * i + 1) = bs(1, i) * Rts[i].row(2) - bs(2, i) * Rts[i].row(1);
  }

  Eigen::JacobiSVD< Eigen::MatrixXd > mySVD(A, Eigen::ComputeFullV );
  Eigen::Vector4d worldPoint;
  worldPoint[0] = mySVD.matrixV()(0,3);
  worldPoint[1] = mySVD.matrixV()(1,3);
  worldPoint[2] = mySVD.matrixV()(2,3);
  worldPoint[3] = mySVD.matrixV()(3,3);

  return worldPoint;
}


bp::object TriangulateBearings(const bp::list &Rts_list,
                       const bp::list &bs_list,
                       double threshold,
                       double min_angle) {

  int n = bp::len(Rts_list);
  vector_mat34 Rts;
  Eigen::Matrix<double, 3, Eigen::Dynamic> bs(3, n);
  Eigen::MatrixXd vs(3, n);
  bool angle_ok = false;
  for (int i = 0; i < n; ++i) {
    bp::object oRt = Rts_list[i];
    bp::object ob = bs_list[i];

    PyArrayContiguousView<double> Rt_array(oRt);
    PyArrayContiguousView<double> b_array(ob);

    Eigen::Map<const Eigen::MatrixXd> Rt(Rt_array.data(), 4, 3);
    Eigen::Map<const Eigen::MatrixXd> b(b_array.data(), 3, 1);

    Rts.push_back(Rt.transpose());
    bs.col(i) = b.col(0);

    // Check angle between rays
    if (!angle_ok) {
      Eigen::Vector3d xh;
      xh << b(0,0), b(1,0), b(2,0);
      Eigen::Vector3d v = Rt.block<3,3>(0,0).transpose().inverse() * xh;
      vs.col(i) << v(0), v(1), v(2);

      for (int j = 0; j < i; ++j) {
        Eigen::Vector3d a, b;
        a << vs(0, i), vs(1, i), vs(2, i);
        b << vs(0, j), vs(1, j), vs(2, j);
        double angle = AngleBetweenVectors(a, b);
        if (angle >= min_angle) {
          angle_ok = true;
        }
      }
    }
  }

  if (!angle_ok) {
    return TriangulateReturn(TRIANGULATION_SMALL_ANGLE, bp::object());
  }

  Eigen::Vector4d X = TriangulateBearingsDLT(bs, Rts);
  X /= X(3);

  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d x_reproj = Rts[i] * X;
    Eigen::Vector3d b;
    b << bs(0, i), bs(1, i), bs(2, i);

    double error = AngleBetweenVectors(x_reproj, b);
    if (error > threshold) {
     return TriangulateReturn(TRIANGULATION_BAD_REPROJECTION, bp::object());
    }
  }

  npy_intp Xe_shape[1] = {3};
  return TriangulateReturn(TRIANGULATION_OK,
      bpn_array_from_data(1, Xe_shape, X.data()));
}


}

