#pragma once
#include <foundation/types.h>

#include <Eigen/Core>
#include <memory>
#include <vector>
namespace map {
struct KeyCompare {
  template <typename T>
  bool operator()(T* lhs, T* rhs) const {
    return lhs->id_ < rhs->id_;
  }
  template <typename T>
  bool operator()(T const* lhs, T const* rhs) const {
    return lhs->id_ < rhs->id_;
  }
};

using RigModelId = std::string;
using RigCameraId = std::string;
using RigInstanceId = size_t;
using ShotId = std::string;
using ShotUniqueId = size_t;
using LandmarkId = std::string;
using LandmarkUniqueId = size_t;
using FeatureId = size_t;
using CameraId = std::string;
}  // namespace map
template <class T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
using DescriptorType = Eigen::Matrix<uint8_t, 1, 32, Eigen::RowMajor>;
using DescriptorMatrix =
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
