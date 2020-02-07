#pragma once

#include "model.h"
#include <foundation/numeric.h>
#include <geometry/essential.h>


class EpipolarSymmetric{
  public:
    static double Error(const Eigen::Matrix3d& model, const Eigen::Vector3d& x, const Eigen::Vector3d& y){
    Eigen::Vector3d E_x = model * x;
    Eigen::Vector3d Et_y = model.transpose() * y;
    return SQUARE(y.dot(E_x)) * ( 1 / E_x.head<2>().squaredNorm()
                                + 1 / Et_y.head<2>().squaredNorm())
      / 4.0;  // The divide by 4 is to make this match the sampson distance
  }
};


class EpipolarGeodesic{
  public:
    static double Error(const Eigen::Matrix3d& model, const Eigen::Vector3d& x, const Eigen::Vector3d& y){
    const double yt_E_x = y.dot(model * x);
    return std::asin(yt_E_x);
  }
};

class EssentialMatrixSolvingFivePoints{
  public:
  template< class IT>
  static std::vector<Eigen::Matrix<double, 3, 3>> Solve(IT begin, IT end){
    return EssentialFivePoints(begin, end);
  }
};

class EssentialMatrixSolvingNPoints{
  public:
  template< class IT>
  static std::vector<Eigen::Matrix<double, 3, 3>> Solve(IT begin, IT end){
    return EssentialNPoints(begin, end);
  }
};

template< class E = EpipolarSymmetric, class S = EssentialMatrixSolvingFivePoints >
class EssentialMatrix : public Model<EssentialMatrix<E, S>, 1, 10, ModelAdapter<EssentialMatrix<E, EssentialMatrixSolvingNPoints> >> {
 public:
  using ERROR = typename Model<EssentialMatrix<E, S>, 1, 10>::ERROR;
  using MODEL = Eigen::Matrix3d;
  using DATA = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 5;

  template <class IT>
  static int Model(IT begin, IT end, MODEL* models){
    const auto essentials = S::Solve(begin, end);
    for(int i = 0; i < essentials.size(); ++i){
      models[i] = essentials[i];
    }
    return essentials.size();
  }

  static ERROR Error(const MODEL& model, const DATA& d){
    const auto x = d.first;
    const auto y = d.second;
    ERROR e;
    e[0] = E::Error(model, x, y);
    return e;
  }
};