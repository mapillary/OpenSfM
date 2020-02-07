#pragma once


#include "model.h"
#include "essential_model.h"
#include <foundation/numeric.h>
#include <geometry/pose.h>


template< class S = EssentialMatrixSolvingFivePoints>
class RelativePose : public Model<RelativePose<S>, 1, 10, ModelAdapter<RelativePose<EssentialMatrixSolvingNPoints> >> {
 public:
  using ERROR = typename Model<RelativePose<S>, 1, 10>::ERROR;
  using MODEL = Eigen::Matrix<double, 3, 4>;
  using DATA = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 5;

  template <class IT>
  static int Model(IT begin, IT end, MODEL* models){
    const auto essentials = S::Solve(begin, end);
    for(int i = 0; i < essentials.size(); ++i){
      models[i] = RelativePoseFromEssential(essentials[i], begin, end);
    }
    return essentials.size();
  }

  static ERROR Error(const MODEL& model, const DATA& d){
    const auto rotation = model.block<3, 3>(0, 0);
    const auto translation = model.block<3, 1>(0, 3);
    const auto x = d.first.normalized();
    const auto y = d.second.normalized();

    Eigen::Matrix<double, 3, 2> bearings;
    Eigen::Matrix<double, 3, 2> centers;
    centers.col(0) << Eigen::Vector3d::Zero();
    centers.col(1) << -rotation.transpose()*translation;
    bearings.col(0) << x;
    bearings.col(1) << rotation.transpose()*y;
    const auto point = csfm::TriangulateBearingsMidpointSolve(centers, bearings);
    const auto projected_x = point.normalized();
    const auto projected_y = (rotation*point+translation).normalized();

    ERROR e;
    e[0] = std::acos((projected_x.dot(x) + projected_y.dot(y))*0.5);
    return e;
  }
};