#pragma once

#include <bundle/data/data.h>
#include <geometry/similarity.h>

namespace bundle {

struct Bias : public Data<geometry::Similarity> {
  enum Parameter { RX, RY, RZ, TX, TY, TZ, SCALE, NUM_PARAMS };

  Bias(const std::string &id, const geometry::Similarity &value)
      : Data<geometry::Similarity>(id, value) {
    Init();
  }

 private:
  void ValueToData(const geometry::Similarity &value, VecXd &data) const final {
    data.resize(NUM_PARAMS);
    data.segment<3>(TX) = value.Translation();
    data.segment<3>(RX) = value.Rotation();
    data(SCALE) = value.Scale();
  }

  void DataToValue(const VecXd &data, geometry::Similarity &value) const final {
    value.SetTranslation(data.segment<3>(TX));
    value.SetRotation(data.segment<3>(RX));
    value.SetScale(data(SCALE));
  }
};
}  // namespace bundle
