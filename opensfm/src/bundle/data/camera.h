#pragma once

#include <bundle/data/data.h>
#include <geometry/camera.h>

namespace bundle {

struct Camera : public Data<::Camera> {
  Camera(const std::string &id, const ::Camera &value, const ::Camera &prior,
         const ::Camera &sigma)
      : Data<::Camera>(id, value, prior, sigma) {
    Init();
  }

 private:
  void ValueToData(const ::Camera &value, VecXd &data) const final {
    data = value.GetParametersValues();
  }

  void DataToValue(const VecXd &data, ::Camera &value) const final {
    value.SetParametersValues(data);
  }
};
}  // namespace bundle
