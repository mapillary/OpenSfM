#pragma once

#include <bundle/data/data.h>
#include <bundle/data/pose.h>
#include <bundle/error/position_functors.h>
#include <geometry/similarity.h>

namespace bundle {

struct Similarity : public Data<geometry::Similarity> {
  enum Parameter { RX, RY, RZ, TX, TY, TZ, SCALE, NUM_PARAMS };

  Similarity(const std::string &id, const geometry::Similarity &value)
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

struct SimilarityPriorTransform
    : public geometry::Functor<Pose::NUM_PARAMS, Similarity::NUM_PARAMS,
                               Pose::NUM_PARAMS> {
  template <typename T>
  VecN<T, Pose::NUM_PARAMS> operator()(T const *parameters,
                                       T const *data) const {
    Vec3<T> R = ShotRotationFunctor(0, FUNCTOR_NOT_SET)(&parameters);
    Vec3<T> t = ShotPositionFunctor(0, FUNCTOR_NOT_SET)(&parameters);
    const T *const scale = parameters + Similarity::Parameter::SCALE;

    VecN<T, Pose::NUM_PARAMS> transformed =
        Eigen::Map<const VecN<T, Pose::NUM_PARAMS>>(data).eval();

    // Apply similarity only to the translation component
    Vec3<T> point = transformed.template segment<3>(Pose::TX);
    point = (scale[0] * RotatePoint(R.eval(), point) + t);
    transformed.template segment<3>(Pose::TX) = point;

    return transformed;
  }
};

}  // namespace bundle
