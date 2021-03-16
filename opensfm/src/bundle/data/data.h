#pragma once

#include <foundation/optional.h>
#include <foundation/types.h>

#include <Eigen/Eigen>
#include <numeric>
#include <unordered_map>

namespace bundle {

struct DataNode {
  explicit DataNode(const std::string &id) : id_(id) {}
  std::string GetID() const { return id_; }

 protected:
  std::string id_;
};

template <class T>
struct Data : public DataNode {
 public:
  using ValueType = T;

  Data(const std::string &id, const T &value, const T &prior, const T &sigma)
      : DataNode(id), value_(value), prior_(prior), sigma_(sigma) {}
  Data(const std::string &id, const T &value) : DataNode(id), value_(value) {}
  virtual ~Data() {}

  VecXd &GetValueData() { return value_data_; }
  T GetValue() const {
    T v = value_;
    DataToValue(value_data_, v);
    return v;
  }

  bool HasPrior() const { return prior_.HasValue(); }
  VecXd GetPriorData() const {
    if (!prior_.HasValue()) {
      throw std::runtime_error(GetID() + " hasn't any prior value");
    }
    VecXd prior_data;
    ValueToData(prior_.Value(), prior_data);
    return prior_data;
  }
  void SetPrior(const T &prior) { prior_.SetValue(prior); }

  VecXd GetSigmaData() const {
    if (!sigma_.HasValue()) {
      throw std::runtime_error(GetID() + " hasn't any sigma value");
    }
    VecXd sigma_data;
    ValueToData(sigma_.Value(), sigma_data);
    return sigma_data;
  }
  void SetSigma(const T &sigma) { sigma_.SetValue(sigma); }

  void SetCovariance(const MatXd &covariance) {
    covariance_.SetValue(covariance);
  }
  bool HasCovariance() const { return covariance_.HasValue(); }
  MatXd GetCovariance() const {
    if (!HasCovariance()) {
      throw std::runtime_error(GetID() + " hasn't any covariance");
    }
    return covariance_.Value();
  }

  const std::vector<int> &GetParametersToOptimize() const {
    return parameters_to_optimize_;
  }
  void SetParametersToOptimize(const std::vector<int> &parameters) {
    parameters_to_optimize_ = parameters;
  }

  virtual void ValueToData(const T &value, VecXd &data) const = 0;
  virtual void DataToValue(const VecXd &data, T &value) const = 0;

 protected:
  VecXd value_data_;
  std::vector<int> parameters_to_optimize_;

  T value_;
  foundation::OptionalValue<T> prior_;
  foundation::OptionalValue<T> sigma_;
  foundation::OptionalValue<MatXd> covariance_;

  void Init() {
    ValueToData(value_, value_data_);
    parameters_to_optimize_.resize(value_data_.size());
    std::iota(parameters_to_optimize_.begin(), parameters_to_optimize_.end(),
              0);
  }
};

struct DataContainer : public DataNode {
  explicit DataContainer(const std::string &id) : DataNode(id) {}

 protected:
  void RegisterData(const std::string &id, DataNode *data) {
    ba_nodes_[id] = data;
  }
  DataNode *GetData(const std::string &id) {
    const auto find_data = ba_nodes_.find(id);
    if (find_data == ba_nodes_.end()) {
      throw std::runtime_error("Data " + id +
                               " doesn't exist in DataContainer");
    }
    return find_data->second;
  }
  const DataNode *GetData(const std::string &id) const {
    const auto find_data = ba_nodes_.find(id);
    if (find_data == ba_nodes_.end()) {
      throw std::runtime_error("Data " + id +
                               " doesn't exist in DataContainer");
    }
    return find_data->second;
  }

 private:
  std::unordered_map<std::string, DataNode *> ba_nodes_;
};
}  // namespace bundle
