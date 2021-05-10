#pragma once

#include <foundation/numeric.h>
#include <foundation/types.h>
#include <geometry/functions.h>

#include <unsupported/Eigen/AutoDiff>

namespace geometry {
/* Parameters are : focal, aspect ratio, cx, cy */
struct Affine : Functor<2, 4, 2> {
  enum { Focal = 0, AspectRatio = 1, Cx = 2, Cy = 3 };

  template <class T>
  static void Forward(const T* point, const T* affine, T* transformed) {
    transformed[0] = affine[Focal] * point[0] + affine[Cx];
    transformed[1] =
        affine[Focal] * affine[AspectRatio] * point[1] + affine[Cy];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* affine,
                                 T* transformed, T* jacobian) {
    // dx, dy, dfocal, daspect_ratio, dcx, dcy
    constexpr int stride = Stride<DERIV_PARAMS>();
    jacobian[0] = affine[Focal];
    jacobian[stride + 1] = affine[Focal] * affine[AspectRatio];
    jacobian[1] = jacobian[stride] = T(0.0);
    if (DERIV_PARAMS) {
      jacobian[2] = point[0];
      jacobian[3] = jacobian[5] = T(0.0);
      jacobian[4] = T(1.0);

      jacobian[stride + 2] = point[1] * affine[AspectRatio];
      jacobian[stride + 3] = point[1] * affine[Focal];
      jacobian[stride + 4] = T(0.0);
      jacobian[stride + 5] = T(1.0);
    }
    Forward(point, affine, transformed);
  }

  template <class T>
  static void Backward(const T* point, const T* affine, T* transformed) {
    transformed[0] = (point[0] - affine[Cx]) / affine[Focal];
    transformed[1] =
        (point[1] - affine[Cy]) / (affine[AspectRatio] * affine[Focal]);
  }
};

/* Parameters are : focal */
struct UniformScale : Functor<2, 1, 2> {
  enum { Focal = 0 };

  template <class T>
  static void Forward(const T* point, const T* scale, T* transformed) {
    transformed[0] = scale[Focal] * point[0];
    transformed[1] = scale[Focal] * point[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* scale, T* transformed,
                                 T* jacobian) {
    // dx, dy, dscale
    constexpr int stride = Stride<DERIV_PARAMS>();
    jacobian[0] = jacobian[stride + 1] = scale[Focal];
    jacobian[1] = jacobian[stride] = T(0.0);
    if (DERIV_PARAMS) {
      jacobian[2] = point[0];
      jacobian[stride + 2] = point[1];
    }
    Forward(point, scale, transformed);
  }

  template <class T>
  static void Backward(const T* point, const T* scale, T* transformed) {
    transformed[0] = point[0] / scale[Focal];
    transformed[1] = point[1] / scale[Focal];
  }
};

/* Parameters are : none used */
struct Identity : Functor<2, 0, 2> {
  template <class T>
  static void Forward(const T* point, const T* /* k */, T* transformed) {
    transformed[0] = point[0];
    transformed[1] = point[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* /* k */,
                                 T* transformed, T* jacobian) {
    // dx, dy
    constexpr int stride = Stride<DERIV_PARAMS>();
    jacobian[0] = jacobian[stride + 1] = T(1.0);
    jacobian[1] = jacobian[stride] = T(0.0);
    T* dummy = nullptr;
    Forward(point, dummy, transformed);
  }

  template <class T>
  static void Backward(const T* point, const T* /* k */, T* transformed) {
    transformed[0] = point[0];
    transformed[1] = point[1];
  }
};

struct PoseFunctor : Functor<3, 6, 3> {
  /* Rotation and translation being stored as angle-axis | translation, apply
   the transformation : x_world = R(t)*(x_camera - translation) by directly
   applying the angle-axis rotation */
  enum { Rx = 0, Ry = 1, Rz = 2, Tx = 3, Ty = 4, Tz = 5 };
  template <class T>
  static void Forward(const T* point, const T* rt, T* transformed) {
    const T x = point[0] - rt[Tx];
    const T y = point[1] - rt[Ty];
    const T z = point[2] - rt[Tz];

    const T a = -rt[Rx];
    const T b = -rt[Ry];
    const T c = -rt[Rz];

    // Dot product of angle-axis and the point
    const T cp_x = b * z - c * y;
    const T cp_y = c * x - a * z;
    const T cp_z = a * y - b * x;

    const T theta2 = a * a + b * b + c * c;
    // Regular angle-axis transformation
    if (theta2 > T(std::numeric_limits<double>::epsilon())) {
      const T theta = sqrt(theta2);
      const T cos_theta = cos(theta);
      const T sin_theta = sin(theta) / theta;
      const T dot_pt_p =
          (a * x + b * y + c * z) * (T(1.0) - cos_theta) / theta2;
      transformed[0] = x * cos_theta + sin_theta * cp_x + a * dot_pt_p;
      transformed[1] = y * cos_theta + sin_theta * cp_y + b * dot_pt_p;
      transformed[2] = z * cos_theta + sin_theta * cp_z + c * dot_pt_p;
      // Apply taylor approximation for small angles
    } else {
      transformed[0] = x + cp_x;
      transformed[1] = y + cp_y;
      transformed[2] = z + cp_z;
    }
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* rt, T* transformed,
                                 T* jacobian) {
    // dx, dy, dz, drx, dry, drz, dtz, dty, dtz
    if (DERIV_PARAMS) {
      using Dual = Eigen::AutoDiffScalar<Vec3d>;

      /* Get jacobian or R wrt. angle-axis using Dual */
      Dual r_diff[InSize];
      r_diff[0].value() = -rt[Rx];
      r_diff[0].derivatives() = -Vec3d::Unit(Rx);
      r_diff[1].value() = -rt[Ry];
      r_diff[1].derivatives() = -Vec3d::Unit(Ry);
      r_diff[2].value() = -rt[Rz];
      r_diff[2].derivatives() = -Vec3d::Unit(Rz);

      Eigen::Matrix<Dual, 3, 3, Eigen::RowMajor> rotation;
      AngleAxisToRotation(&r_diff[0], rotation.data());

      /* Storage is row-ordered : R00, R01, R02, R10, ... R22 */
      Eigen::Matrix<T, 9, 3, Eigen::RowMajor> rotation_angleaxis;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            rotation_angleaxis(i * 3 + j, k) = rotation(i, j).derivatives()(k);
          }
        }
      }

      /* R.(x-t) derivatives are pretty straightfoward : dR00, dR01, ... dR22,
       * dx, dy, dz, dtx, dty, dtz */
      const T xyz[] = {point[0] - rt[Tx], point[1] - rt[Ty], point[2] - rt[Tz]};
      Eigen::Matrix<T, 3, 15, Eigen::RowMajor> point_rotation =
          Eigen::Matrix<T, 3, 15, Eigen::RowMajor>::Zero();
      for (int i = 0; i < 3; ++i) {
        // dRij
        for (int j = 0; j < 3; ++j) {
          point_rotation(i, i * 3 + j) = xyz[j];
        }
        // dx, dy, dz
        for (int j = 0; j < 3; ++j) {
          point_rotation(i, 9 + j) = rotation(i, j).value();
        }
        // dtx, dty, dtz
        for (int j = 0; j < 3; ++j) {
          point_rotation(i, 12 + j) = -point_rotation(i, 9 + j);
        }
      }

      /* Compose d(R) / d(angle axis) with d(pose) / d(R | t | x) in order to
       * get d(pose) / d(angle axis | t | x) */

      ComposeDerivatives<T, true, 9, 3, 0, 3, 15, 6>(rotation_angleaxis,
                                                     point_rotation, jacobian);

      /* Re-orde from angle-axis | x | t to x | angle-axis | t */
      for (int i = 0; i < 3; ++i) {
        // Swap dai and dxi
        for (int j = 0; j < 3; ++j) {
          std::swap(jacobian[i * 9 + j], jacobian[i * 9 + 3 + j]);
        }
      }
    } else {
      double minus_r[] = {-rt[Rx], -rt[Ry], -rt[Rz]};
      AngleAxisToRotation(&minus_r[0], jacobian);
    }
    Forward(point, rt, transformed);
  }

 private:
  template <class T>
  static void AngleAxisToRotation(const T* angle_axis, T* rotation) {
    const T theta2 = SquaredNorm(angle_axis) + angle_axis[2] * angle_axis[2];

    // Use Taylor approximation near zero angle : R = I + [angle_axis]x
    if (theta2 < T(std::numeric_limits<double>::epsilon())) {
      Eigen::Map<Mat3<T>> mapped_rotation(rotation);
      const Eigen::Map<const Vec3<T>> mapped_angle_axis(angle_axis);
      foundation::SkewMatrixT(mapped_angle_axis, &mapped_rotation);
      for (int i = 0; i < 3; ++i) {
        rotation[i * 3 + i] = T(1.0);
      }
      // From
      // https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
    } else {
      const T theta = sqrt(theta2);
      const T c = cos(theta);
      const T s = sin(theta);
      const T t = T(1.0) - c;

      const T inv_theta2 = T(1.0) / theta2;
      const T inv_theta = T(1.0) / theta;

      const T xx = angle_axis[0] * angle_axis[0] * inv_theta2;
      const T xy = angle_axis[0] * angle_axis[1] * inv_theta2;
      const T xz = angle_axis[0] * angle_axis[2] * inv_theta2;
      const T yy = angle_axis[1] * angle_axis[1] * inv_theta2;
      const T yz = angle_axis[1] * angle_axis[2] * inv_theta2;
      const T zz = angle_axis[2] * angle_axis[2] * inv_theta2;

      const T xs = angle_axis[0] * inv_theta * s;
      const T ys = angle_axis[1] * inv_theta * s;
      const T zs = angle_axis[2] * inv_theta * s;

      rotation[0] = t * xx + c;
      rotation[1] = t * xy - zs;
      rotation[2] = t * xz + ys;

      rotation[3] = t * xy + zs;
      rotation[4] = t * yy + c;
      rotation[5] = t * yz - xs;

      rotation[6] = t * xz - ys;
      rotation[7] = t * yz + xs;
      rotation[8] = t * zz + c;
    }
  }
};

struct Normalize : Functor<3, 0, 3> {
  template <class T>
  static void Forward(const T* point, const T* /* k */, T* transformed) {
    const T inv_norm = T(1.0) / sqrt(SquaredNorm(point) + point[2] * point[2]);
    for (int i = 0; i < 3; ++i) {
      transformed[i] = point[i] * inv_norm;
    }
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* /* k */,
                                 T* transformed, T* jacobian) {
    // dx, dy
    constexpr int stride = Stride<DERIV_PARAMS>();

    const T& x = point[0];
    const T& y = point[1];
    const T& z = point[2];
    const T x2 = x * x;
    const T y2 = y * y;
    const T z2 = z * z;
    const T norm2 = x2 + y2 + z2;
    const T norm = sqrt(norm2);
    const T inv_norm32 = T(1.0) / (norm * norm2);

    jacobian[0] = (y2 + z2) * inv_norm32;
    jacobian[1] = (-x * y) * inv_norm32;
    jacobian[2] = (-x * z) * inv_norm32;

    jacobian[stride] = (-y * x) * inv_norm32;
    jacobian[stride + 1] = (x2 + z2) * inv_norm32;
    jacobian[stride + 2] = (-y * z) * inv_norm32;

    jacobian[2 * stride] = (-z * x) * inv_norm32;
    jacobian[2 * stride + 1] = (-z * y) * inv_norm32;
    jacobian[2 * stride + 2] = (x2 + y2) * inv_norm32;

    T* dummy = nullptr;
    Forward(point, dummy, transformed);
  }
};

static Mat3d VectorToRotationMatrix(const Vec3d& r) {
  const auto n = r.norm();
  if (n == 0)  // avoid division by 0
  {
    return Eigen::AngleAxisd(0, r).toRotationMatrix();
  } else {
    return Eigen::AngleAxisd(n, r / n).toRotationMatrix();
  }
}
static Vec3d RotationMatrixToVector(const Mat3d& R) {
  Eigen::AngleAxisd tmp(R);
  return tmp.axis() * tmp.angle();
}
}  // namespace geometry
