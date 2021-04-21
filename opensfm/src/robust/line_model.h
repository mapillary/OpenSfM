#pragma once

#include "model.h"

class Line : public Model<Line, 1, 1> {
 public:
  using Type = Eigen::Vector2d;
  using Data = Eigen::Vector2d;
  static const int MINIMAL_SAMPLES = 2;

  template <class IT>
  static int Estimate(IT begin, IT end, Type* models) {
    const auto x1 = *begin;
    const auto x2 = *(++begin);
    const auto b = (x1[0] * x2[1] - x1[1] * x2[0]) / (x1[0] - x2[0]);

    if (x1[0] == 0.0) {
      return 0;
    }

    const auto a = (x1[1] - b) / x1[0];

    if (std::isnan(a) || std::isnan(b)) {
      return 0;
    }

    models[0] << a, b;
    return 1;
  }

  template <class IT>
  static int EstimateNonMinimal(IT begin, IT end, Type* models) {
    return Estimate(begin, end, models);
  }

  static Error Evaluate(const Type& model, const Data& d) {
    const auto a = model[0];
    const auto b = model[1];
    Error e;
    e << d[1] - (a * d[0] + b);
    return e;
  }
};
