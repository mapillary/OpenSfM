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

using RigCameraId = std::string;
using RigInstanceId = size_t;
using ShotId = std::string;
using ShotUniqueId = size_t;
using TrackId = std::string;
using LandmarkId = std::string;
using LandmarkUniqueId = size_t;
using FeatureId = size_t;
using CameraId = std::string;
using CameraUniqueId = size_t;
}  // namespace map
template <class T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
using DescriptorType = Eigen::Matrix<uint8_t, 1, 32, Eigen::RowMajor>;
using DescriptorMatrix =
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct HashPair {
  template <class T1, class T2>
  size_t operator()(const std::pair<T1, T2>& p) const {
    auto hash1 = std::hash<T1>{}(p.first);
    auto hash2 = std::hash<T2>{}(p.second);
    return hash1 ^ hash2;
  }
};
