#pragma once

#include <bundle/data/data.h>
#include <geometry/pose.h>

namespace bundle {

struct Pose : public Data<geometry::Pose> {
  // Outputted parametrization : CAM_TO_WORLD is preferred
  enum Parametrization {
    CAM_TO_WORLD = 0,  // x(cam) = Rt*(x(world) - t))
    WORLD_TO_CAM = 1,  // x(cam) = R*x(world) + t
  };

  enum Parameter { RX, RY, RZ, TX, TY, TZ, NUM_PARAMS };

  Pose(const std::string &id, const geometry::Pose &value,
       const Parametrization &parametrization = Parametrization::CAM_TO_WORLD)
      : Data<geometry::Pose>(id, value), parametrization_(parametrization) {
    Init();
  }

  Pose(const std::string &id, const geometry::Pose &value,
       const geometry::Pose &prior, const geometry::Pose &sigma,
       const Parametrization &parametrization = Parametrization::CAM_TO_WORLD)
      : Data<geometry::Pose>(id, value, prior, sigma),
        parametrization_(parametrization) {
    Init();
  }

 private:
  void ValueToData(const geometry::Pose &value, VecXd &data) const final {
    data.resize(NUM_PARAMS);
    if (parametrization_ == Parametrization::CAM_TO_WORLD) {
      data.segment<3>(TX) = value.TranslationCameraToWorld();
      data.segment<3>(RX) = value.RotationCameraToWorldMin();
    } else {
      data.segment<3>(TX) = value.TranslationWorldToCamera();
      data.segment<3>(RX) = value.RotationWorldToCameraMin();
    }
  }

  void DataToValue(const VecXd &data, geometry::Pose &value) const final {
    if (parametrization_ == Parametrization::CAM_TO_WORLD) {
      value.SetFromCameraToWorld(Vec3d(data.segment<3>(RX)),
                                 data.segment<3>(TX));
    } else {
      value.SetFromWorldToCamera(Vec3d(data.segment<3>(RX)),
                                 data.segment<3>(TX));
    }
  }

  Parametrization parametrization_;
};
}  // namespace bundle
