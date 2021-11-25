#pragma once

#include <bundle/data/data.h>

#include "foundation/types.h"

namespace bundle {

struct Point : public Data<Vec3d> {
  enum Parameter { PX, PY, PZ, NUM_PARAMS };

  Point(const std::string &id, const Vec3d &value) : Data<Vec3d>(id, value) {
    Init();
  }

  Point(const std::string &id, const Vec3d &value, const Vec3d &prior,
        const Vec3d &sigma)
      : Data<Vec3d>(id, value, prior, sigma) {
    Init();
  }

  std::map<std::string, VecXd> reprojection_errors;
  bool has_altitude_prior{true};

 private:
  void ValueToData(const Vec3d &value, VecXd &data) const final {
    data = value;
  }

  void DataToValue(const VecXd &data, Vec3d &value) const final {
    value = data;
  }
};
}  // namespace bundle
