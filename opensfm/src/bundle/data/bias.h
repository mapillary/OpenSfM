#pragma once

#include <bundle/data/data.h>
#include <geometry/pose.h>

namespace bundle {

struct Bias : public Data<geometry::ScaledPose> {

  enum Parameter { RX, RY, RZ, TX, TY, TZ, SCALE, NUM_PARAMS };

  Bias(const std::string &id, const geometry::ScaledPose &value)
      : Data<geometry::ScaledPose>(id, value)
      {
    Init();
  }

 private:
  void ValueToData(const geometry::ScaledPose &value, VecXd &data) const final {
    data.resize(NUM_PARAMS);
    data.segment<3>(TX) = value.TranslationCameraToWorld();
    data.segment<3>(RX) = value.RotationCameraToWorldMin();
    data(SCALE) = value.scale;
  }

  void DataToValue(const VecXd &data, geometry::ScaledPose &value) const final {
    value.SetFromCameraToWorld(Vec3d(data.segment<3>(RX)),
                                data.segment<3>(TX));
    value.scale = data(SCALE);
  }
};
}  // namespace bundle
