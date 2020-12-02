#pragma once

#include <Eigen/Eigen>
#include <vector>

template <class T, int N, int M>
class Model {
 public:
  static const int SIZE = N;
  static const int MAX_MODELS = M;
  using Error = Eigen::Matrix<double, SIZE, 1>;

  static double ThresholdAdapter(const double threshold) { return threshold; }

  template <class IT, class MODEL>
  static std::vector<Error> EvaluateModel(const MODEL& model, IT begin,
                                          IT end) {
    std::vector<Error> errors;
    std::for_each(begin, end, [&errors, &model](const typename T::Data& d) {
      errors.push_back(T::Evaluate(model, d));
    });
    return errors;
  }
};
