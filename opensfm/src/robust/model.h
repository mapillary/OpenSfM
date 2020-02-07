#pragma once

#include <Eigen/Eigen>
#include <vector>

template< class T >
class ModelAdapter {
  public:
  
  using MODEL = typename T::MODEL;

  template <class IT>
  static int Model(IT begin, IT end, typename T::MODEL* models){
    return T::Model(begin, end, models);
  }

  static typename T::ERROR Error(const typename T::MODEL& model, const typename T::DATA& d){
    return T::Model(model, d);
  }

  template <class IT>
  static std::vector<typename T::ERROR> Errors(const typename T::MODEL& model, IT begin, IT end) {
    return T::Errors(model, begin, end);
  }
};

template <class T, int N, int M, class L = ModelAdapter<T> >
class Model {
 public:
  static const int SIZE = N;
  static const int MAX_MODELS = M;
  using ERROR = Eigen::Matrix<double, SIZE, 1>;
  using LOMODEL = L;

  template <class IT, class MODEL>
  static std::vector<ERROR> Errors(const MODEL& model, IT begin, IT end) {
    std::vector<ERROR> errors;
    std::for_each(begin, end, [&errors, &model](const typename T::DATA& d) {
      errors.push_back(T::Error(model, d));
    });
    return errors;
  }
};