#pragma once

#include <bundle/bundle_adjuster.h>
#include <bundle/error/error_utils.h>
#include <ceres/sized_cost_function.h>
#include <foundation/types.h>
#include <geometry/functions.h>
#include <geometry/camera_instances.h>

namespace bundle {
class RelativeDepthError {
 public:
  constexpr static int Size = 1;
  RelativeDepthError(double depth, double std_deviation, bool use_rig_camera, bool is_radial_depth = true)
      : depth_(depth),
        scale_(1.0 / std_deviation),
        use_rig_camera_(use_rig_camera),
        is_radial_depth_(is_radial_depth) {}

  template <typename T>
  bool operator()(const T* const rig_instance,
                  const T* const rig_camera, const T* const point,
                  T* residuals) const {
    T scale_one = T(1.0);
    Vec3<T> point_in_camera;
    const T* const actual_rig_camera = use_rig_camera_ ? rig_camera : nullptr;
    WorldToCameraCoordinatesRig(&scale_one, rig_instance, actual_rig_camera,
                                point, point_in_camera.data());

    // The error is the difference between the predicted and observed depth
    T depth_in_camera = point_in_camera[2];
    if (is_radial_depth_) {
      depth_in_camera = point_in_camera.norm();
    }
    residuals[0] = T(scale_) * (depth_in_camera - T(depth_));

    return true;
  }

 protected:
  const double depth_;
  const double scale_;
  const bool use_rig_camera_;
  const bool is_radial_depth_;
};
}  // namespace bundle
