#pragma once

#include <bundle/data/data.h>
#include <geometry/camera.h>

namespace bundle {

struct Camera : public Data<geometry::Camera> {
  Camera(const std::string &id, const geometry::Camera &value,
         const geometry::Camera &prior, const geometry::Camera &sigma)
      : Data<geometry::Camera>(id, value, prior, sigma) {
    Init();
  }

 private:
  void ValueToData(const geometry::Camera &value, VecXd &data) const final {
    data = value.GetParametersValues();
  }

  void DataToValue(const VecXd &data, geometry::Camera &value) const final {
    value.SetParametersValues(data);
  }
};
}  // namespace bundle
