#pragma once

#include <geometry/transform.h>
#include <robust/model.h>

class Similarity : public Model<Similarity, 1, 1> {
 public:
  using Type = Eigen::Matrix4d;
  using Data = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 3;

  template <class IT>
  static int Estimate(IT begin, IT end, Type* models) {
    models[0] << SimilarityBetweenPoints(begin, end);
    return 1;
  }

  template <class IT>
  static int EstimateNonMinimal(IT begin, IT end, Type* models) {
    return Estimate(begin, end, models);
  }

  static Error Evaluate(const Type& model, const Data& d) {
    Vec4d homogenous(d.first[0], d.first[1], d.first[2], 1.0);
    return Error((d.second - (model * homogenous).segment<3>(0)).norm());
  }
};
