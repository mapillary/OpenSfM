#pragma once

#include <foundation/numeric.h>
#include <geometry/essential.h>
#include "model.h"

class EpipolarSymmetric {
 public:
  static double Error(const Eigen::Matrix3d& model, const Eigen::Vector3d& x,
                      const Eigen::Vector3d& y) {
    Eigen::Vector3d E_x = model * x;
    Eigen::Vector3d Et_y = model.transpose() * y;
    return SQUARE(y.dot(E_x)) *
           (1 / E_x.head<2>().squaredNorm() +
            1 / Et_y.head<2>().squaredNorm()) /
           4.0;  // The divide by 4 is to make this match the sampson distance
  }
};

class EpipolarGeodesic {
 public:
  static double Error(const Eigen::Matrix3d& model, const Eigen::Vector3d& x,
                      const Eigen::Vector3d& y) {
    const double yt_E_x = y.dot(model * x);
    return std::asin(yt_E_x);
  }
};

template <class E = EpipolarSymmetric>
class EssentialMatrix : public Model<EssentialMatrix<E>, 1, 10> {
 public:
  using Error = typename Model<EssentialMatrix<E>, 1, 10>::Error;
  using Type = Eigen::Matrix3d;
  using Data = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 5;

  template <class IT>
  static int Estimate(IT begin, IT end, Type* models) {
    const auto essentials = EssentialFivePoints(begin, end);
    for (int i = 0; i < essentials.size(); ++i) {
      models[i] = essentials[i];
    }
    return essentials.size();
  }

  template <class IT>
  static int EstimateNonMinimal(IT begin, IT end, Type* models) {
    const auto essentials = EssentialNPoints(begin, end);
    for (int i = 0; i < essentials.size(); ++i) {
      models[i] = essentials[i];
    }
    return essentials.size();
  }

  static Error Evaluate(const Type& model, const Data& d) {
    const auto x = d.first;
    const auto y = d.second;
    Error e;
    e[0] = E::Error(model, x, y);
    return e;
  }
};

template <class E>
const int EssentialMatrix<E>::MINIMAL_SAMPLES;
