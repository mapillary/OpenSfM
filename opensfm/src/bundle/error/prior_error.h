#pragma once

#include <bundle/data/data.h>
#include <foundation/optional.h>
#include <geometry/functions.h>

#include <unordered_set>

namespace bundle {
template <int IN, int P, int OUT>
struct DummyPriorTransform : public geometry::Functor<IN, P, OUT> {
  template <typename T>
  VecN<T, OUT> operator()(T const*, T const*) const {
    return VecN<T, OUT>();
  }
};
using NoTransform = DummyPriorTransform<0, 0, 0>;

template <class DATA, class TRANSFORM = NoTransform>
struct DataPriorError {
 public:
  // Parameter scaling
  enum class ScaleType { LINEAR = 0, LOGARITHMIC = 1 };

  explicit DataPriorError(Data<DATA>* data, bool adjust_scales = false)
      : data_(data), adjust_scales_(adjust_scales) {
    const auto sigma = data->GetSigmaData();
    for (int i = 0; i < sigma.rows(); ++i) {
      indexes_.insert(i);
    }

    scales_.resize(sigma.rows(), 1);
    for (const auto& index : indexes_) {
      scales_(index) =
          1.0 / std::max(sigma(index), std::numeric_limits<double>::epsilon());
    }
  }

  void SetTransform(const TRANSFORM& transform) {
    transform_.SetValue(transform);
  }
  void SetScaleType(int index, const ScaleType& type) {
    if (indexes_.find(index) == indexes_.end()) {
      throw std::runtime_error("Parameter index out-of-range");
    }
    scale_types_[index] = type;
  }

  void SetConstrainedDataIndexes(const std::vector<int>& indexes) {
    indexes_.clear();
    indexes_.insert(indexes.begin(), indexes.end());
  }

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    const auto residual_size = indexes_.size();
    const auto parameter_size = data_->GetSigmaData().rows();

    Eigen::Map<VecX<T>> residuals_mapped(r, residual_size);
    VecX<T> parameters_values =
        Eigen::Map<const VecX<T>>(p[parameter_index], parameter_size);

    VecX<T> prior_values = data_->GetPriorData().template cast<T>();
    if (transform_.HasValue()) {
      prior_values =
          transform_.Value()(p[transform_index], prior_values.data());
    }

    int residual_index = 0;
    for (const auto& index : indexes_) {
      auto scale_type = ScaleType::LINEAR;
      const auto scale_type_find = scale_types_.find(index);
      if (scale_type_find != scale_types_.end()) {
        scale_type = scale_type_find->second;
      }

      T error = T(0.);
      switch (scale_type) {
        case ScaleType::LINEAR:
          error = parameters_values(index) - prior_values(index);
          break;
        case ScaleType::LOGARITHMIC:
          error = log(parameters_values(index) / prior_values(index));
          break;
      }
      T scale = T(scales_(index));
      if (adjust_scales_) {
        scale /= p[scale_index][0];
      }
      residuals_mapped(residual_index++) = scale * error;
    }
    return true;
  }

 private:
  // Data block
  Data<DATA>* data_;

  // Optional transform applied to the prior
  foundation::OptionalValue<TRANSFORM> transform_;

  // Parameters over which to apply the prior
  std::unordered_set<int> indexes_;

  // Prior scaling type (default is linear)
  std::unordered_map<int, ScaleType> scale_types_;
  VecXd scales_;

  // Are scale being adjusted for (global multiplier) ?
  const bool adjust_scales_;
  static constexpr int parameter_index = 0;
  static constexpr int transform_index = 1;
  static constexpr int scale_index = 2;
};
}  // namespace bundle
