#pragma once

#include <foundation/newton_raphson.h>

#include <Eigen/Eigen>
#include <iostream>

class PerspectiveCamera {
 public:
  PerspectiveCamera(double focal, double k1, double k2)
      : focal_(focal), k1_(k1), k2_(k2) {}

  Eigen::Vector2d Project(const Eigen::Vector3d& point) const {
    Eigen::Vector2d projected(point[0] / point[2], point[1] / point[2]);
    const auto r2 = projected.dot(projected);
    const auto distortion = Distorsion(r2);
    return focal_ * projected * distortion;
  }

  Eigen::MatrixX2d ProjectMany(const Eigen::MatrixX3d& points) const {
    Eigen::MatrixX2d projected(points.rows(), 2);
    for (int i = 0; i < points.rows(); ++i) {
      projected.row(i) = Project(points.row(i));
    }
    return projected;
  }

  Eigen::Vector3d Bearing(const Eigen::Vector2d& point) const {
    Eigen::Vector2d bearing(point[0] / focal_, point[1] / focal_);
    // We will initialize with current radius
    const auto rd = bearing.norm();

    // Now, refine with a bit of Newton
    const int iterations = 20;
    struct DistoEval {
      const PerspectiveCamera& self;
      const double rd;
      double operator()(const double& x) const {
        const auto r = x;
        const auto r2 = r * r;
        return r * self.Distorsion(r2) - rd;
      }
      double derivative(const double& x) const {
        const auto r = x;
        const auto r2 = r * r;
        return self.DistorsionDerivative(r2);
      }
    };
    DistoEval eval_function{*this, rd};
    const auto ru_refined =
        NewtonRaphson<DistoEval, 1, 1, ManualDiff<DistoEval, 1, 1>>(
            eval_function, rd, iterations);

    // Finally, compute distorsion factor
    const auto r2 = ru_refined * ru_refined;
    const auto distorsion = Distorsion(r2);

    return Eigen::Vector3d(bearing[0] / distorsion, bearing[1] / distorsion,
                           1.0)
        .normalized();
  }

  Eigen::MatrixX3d BearingsMany(const Eigen::MatrixX2d& points) const {
    Eigen::MatrixX3d projected(points.rows(), 3);
    for (int i = 0; i < points.rows(); ++i) {
      projected.row(i) = Bearing(points.row(i));
    }
    return projected;
  }

  double Distorsion(double r2) const { return 1.0 + r2 * (k1_ + k2_ * r2); }
  double DistorsionDerivative(double r2) const {
    return 1.0 + r2 * 2.0 * (k1_ + 2.0 * k2_ * r2);
  }

  double focal_;
  double k1_;
  double k2_;
};

class BrownCamera {
 public:
  BrownCamera(double focal_x, double focal_y,
              const Eigen::Vector2d& principal_point,
              const Eigen::Vector3d& radial_distorsion,
              const Eigen::Vector2d& tangential_distorsion)
      : focal_x_(focal_x),
        focal_y_(focal_y),
        principal_point_(principal_point),
        radial_distorsion_(radial_distorsion),
        tangential_distorsion_(tangential_distorsion) {}

  Eigen::Vector2d Project(const Eigen::Vector3d& point) {
    Eigen::Vector2d projected(point[0] / point[2], point[1] / point[2]);

    const auto r2 = projected.dot(projected);
    const auto distortion_radial = RadialDistorsion(r2);
    const auto distortion_tangential =
        TangentialDistorsion(r2, projected[0], projected[1]);

    Eigen::Matrix2d affine;
    affine << focal_x_, 0, 0, focal_y_;
    projected = projected * distortion_radial + distortion_tangential;
    return affine * projected + principal_point_;
  }

  Eigen::MatrixX2d ProjectMany(const Eigen::MatrixX3d& points) {
    Eigen::MatrixX2d projected(points.rows(), 2);
    for (int i = 0; i < points.rows(); ++i) {
      projected.row(i) = Project(points.row(i));
    }
    return projected;
  }

  Eigen::Vector3d Bearing(const Eigen::Vector2d& point) {
    Eigen::Vector2d bearing((point[0] - principal_point_[0]) / focal_x_,
                            (point[1] - principal_point_[1]) / focal_y_);

    // Now, refine with a bit of Newton
    const int iterations = 20;
    struct DistoEval {
      const BrownCamera& self;
      const Eigen::Vector2d point_distorted;
      Eigen::Vector2d operator()(const Eigen::Vector2d& point) const {
        const auto r2 = point.dot(point);
        const auto distortion_radial = self.RadialDistorsion(r2);
        const auto distortion_tangential =
            self.TangentialDistorsion(r2, point[0], point[1]);
        return point * distortion_radial + distortion_tangential -
               point_distorted;
      }
      Eigen::Matrix2d derivative(const Eigen::Vector2d& point) const {
        const auto a = self.radial_distorsion_[0];
        const auto b = self.radial_distorsion_[1];
        const auto c = self.radial_distorsion_[2];
        const auto d = self.tangential_distorsion_[0];
        const auto e = self.tangential_distorsion_[1];
        const auto x = point[0];
        const auto y = point[1];
        const auto r2 = point.squaredNorm();
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double x2 = x * x;
        const double x4 = x2 * x2;
        const double y2 = y * y;
        const double y4 = y2 * y2;

        const auto dxx = 5 * b * x4 + 3 * a * x2 + 6 * c * x2 * r4 +
                         6 * b * x2 * y2 + c * r6 + b * y4 + a * y2 + 1 +
                         2 * d * y + 6 * e * x;
        const auto dxy = x * (2 * a * y + 4 * b * y * r2 + 6 * c * y * r4) +
                         2 * d * x + 2 * e * y;

        const auto dyy = 5 * b * y4 + 3 * a * y2 + 6 * c * y2 * r4 +
                         6 * b * x2 * y2 + c * r6 + b * x4 + a * x2 + 1 +
                         2 * e * x + 6 * d * y;
        const auto dyx = y * (2 * a * x + 4 * b * x * r2 + 6 * c * x * r4) +
                         2 * e * y + 2 * d * x;
        Eigen::Matrix2d jacobian;
        jacobian << dxx, dxy, dyx, dyy;
        return jacobian;
      }
    };
    DistoEval eval_function{*this, bearing};
    const auto point_undistorted =
        NewtonRaphson<DistoEval, 2, 2, ManualDiff<DistoEval, 2, 2>>(
            eval_function, bearing, iterations);

    return Eigen::Vector3d(point_undistorted[0], point_undistorted[1], 1.0)
        .normalized();
  }

  Eigen::MatrixX3d BearingsMany(const Eigen::MatrixX2d& points) {
    Eigen::MatrixX3d projected(points.rows(), 3);
    for (int i = 0; i < points.rows(); ++i) {
      projected.row(i) = Bearing(points.row(i));
    }
    return projected;
  }

  double RadialDistorsion(double r2) const {
    return 1.0 +
           r2 * (radial_distorsion_[0] +
                 r2 * (radial_distorsion_[1] + r2 * radial_distorsion_[2]));
  }
  Eigen::Vector2d TangentialDistorsion(double r2, double x, double y) const {
    return Eigen::Vector2d(2.0 * tangential_distorsion_[0] * x * y +
                               tangential_distorsion_[1] * (r2 + 2 * x * x),
                           2.0 * tangential_distorsion_[1] * x * y +
                               tangential_distorsion_[0] * (r2 + 2 * y * y));
  }

  double focal_x_;
  double focal_y_;
  Eigen::Vector2d principal_point_;
  Eigen::Vector3d radial_distorsion_;
  Eigen::Vector2d tangential_distorsion_;
};
