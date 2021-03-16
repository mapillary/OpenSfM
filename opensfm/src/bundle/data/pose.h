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

  enum Parameter {
    BA_SHOT_RX,
    BA_SHOT_RY,
    BA_SHOT_RZ,
    BA_SHOT_TX,
    BA_SHOT_TY,
    BA_SHOT_TZ,
    BA_SHOT_NUM_PARAMS
  };

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
    data.resize(BA_SHOT_NUM_PARAMS);
    if (parametrization_ == Parametrization::CAM_TO_WORLD) {
      data.segment<3>(BA_SHOT_TX) = value.TranslationCameraToWorld();
      data.segment<3>(BA_SHOT_RX) = value.RotationCameraToWorldMin();
    } else {
      data.segment<3>(BA_SHOT_TX) = value.TranslationWorldToCamera();
      data.segment<3>(BA_SHOT_RX) = value.RotationWorldToCameraMin();
    }
  }

  void DataToValue(const VecXd &data, geometry::Pose &value) const final {
    if (parametrization_ == Parametrization::CAM_TO_WORLD) {
      value.SetFromCameraToWorld(Vec3d(data.segment<3>(BA_SHOT_RX)),
                                 data.segment<3>(BA_SHOT_TX));
    } else {
      value.SetFromWorldToCamera(Vec3d(data.segment<3>(BA_SHOT_RX)),
                                 data.segment<3>(BA_SHOT_TX));
    }
  }

  Parametrization parametrization_;
};
}  // namespace bundle
