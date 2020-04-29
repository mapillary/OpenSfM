#pragma once

#include <Eigen/Eigen>
#include <iostream>

#include <foundation/newton_raphson.h>

class PerspectiveCamera {
 public:
  PerspectiveCamera(double focal, double k1, double k2)
      : focal_(focal), k1_(k1), k2_(k2) {}

  Eigen::Vector2d Project(const Eigen::Vector3d& point) {
    Eigen::Vector2d projected(point[0] / point[2], point[1] / point[2]);
    const auto r2 =  projected.dot(projected);
    const auto distortion = 1.0 + r2 * (k1_ + k2_ * r2);
    return focal_*projected*distortion;
  }

  Eigen::MatrixX2d ProjectMany(const Eigen::MatrixX3d& points) {
    Eigen::MatrixX2d projected(points.rows(), 2);
    for(int i = 0; i < points.rows(); ++i){
      projected.row(i) = Project(points.row(i));
    }
    return projected;
  }

  Eigen::Vector3d Bearing(const Eigen::Vector2d& point) {
    Eigen::Vector2d bearing(point[0]/focal_, point[1]/focal_);
    // Initialize with current radius
    const auto rd = bearing.norm();

    // Now, refine with a bit of Newton
    const int iterations = 20;
    struct DistoEval{
      const double k1;
      const double k2;
      const double rd;
      double operator()(double x)const{
        const auto r2 = x*x;
        return x*(1.0 + r2 * (k1 + k2 * r2)) - rd;
      }
      double derivative(double x)const{
        const auto r2 = x*x;
        return 1.0 + r2*2.0*(k1 + 2.0*k2 * r2);
      }
    };
    DistoEval eval_function{k1_, k2_, rd};
    const auto ru_refined = NewtonRaphson<DistoEval, ManualDiff<DistoEval> >(eval_function, rd, iterations);

    // Finally, compute distorsion factor
    const auto r2 = ru_refined*ru_refined;
    const auto distorsion = 1.0 + r2 * (k1_ + k2_ * r2);

    return Eigen::Vector3d(bearing[0]/distorsion, bearing[1]/distorsion, 1.0).normalized();
  }

  Eigen::MatrixX3d BearingsMany(const Eigen::MatrixX2d& points) {
    Eigen::MatrixX3d projected(points.rows(), 3);
    for(int i = 0; i < points.rows(); ++i){
      projected.row(i) = Bearing(points.row(i));
    }
    return projected;
  }

  double focal_;
  double k1_;
  double k2_;
};
