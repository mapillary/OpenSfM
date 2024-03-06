#include <features/matching.h>
#include <foundation/optional.h>
#include <foundation/types.h>
#include <pybind11/pybind11.h>

#include <cassert>
#include <limits>
#include <map>
#include <opencv2/core/core.hpp>
#include <stdexcept>
#include <vector>

namespace py = pybind11;

namespace features {

float DistanceL1(const float *pa, const float *pb, int n) {
  float distance = 0;
  for (int i = 0; i < n; ++i) {
    distance += fabs(pa[i] - pb[i]);
  }
  return distance;
}

float DistanceL2(const float *pa, const float *pb, int n) {
  float distance = 0;
  for (int i = 0; i < n; ++i) {
    distance += (pa[i] - pb[i]) * (pa[i] - pb[i]);
  }
  return sqrt(distance);
}

void MatchUsingWords(const cv::Mat &f1, const cv::Mat &w1, const cv::Mat &f2,
                     const cv::Mat &w2, float lowes_ratio, int max_checks,
                     cv::Mat *matches) {
  // Index features on the second image.
  std::multimap<int, int> index2;
  const int *pw2 = &w2.at<int>(0, 0);
  for (unsigned int i = 0; i < w2.rows * w2.cols; ++i) {
    index2.insert(std::pair<int, int>(pw2[i], i));
  }

  std::vector<int> best_match(f1.rows, -1), second_best_match(f1.rows, -1);
  std::vector<float> best_distance(f1.rows, std::numeric_limits<float>::infinity());
  std::vector<float> second_best_distance(f1.rows, std::numeric_limits<float>::infinity());
  *matches = cv::Mat(0, 2, CV_32S);
  cv::Mat tmp_match(1, 2, CV_32S);
  for (unsigned int i = 0; i < w1.rows; ++i) {
    int checks = 0;
    for (unsigned int j = 0; j < w1.cols; ++j) {
      int word = w1.at<int>(i, j);
      auto range = index2.equal_range(word);
      for (auto it = range.first; it != range.second; ++it) {
        int match = it->second;
        const float *pa = f1.ptr<float>(i);
        const float *pb = f2.ptr<float>(match);
        float distance = DistanceL2(pa, pb, f1.cols);
        if (distance < best_distance[i]) {
          second_best_distance[i] = best_distance[i];
          second_best_match[i] = best_match[i];
          best_distance[i] = distance;
          best_match[i] = match;
        } else if (distance < second_best_distance[i]) {
          second_best_distance[i] = distance;
          second_best_match[i] = match;
        }
        checks++;
      }
      if (checks >= max_checks) {
        break;
      }
    }
    if (best_distance[i] < lowes_ratio * second_best_distance[i]) {
      tmp_match.at<int>(0, 0) = i;
      tmp_match.at<int>(0, 1) = best_match[i];
      matches->push_back(tmp_match);
    }
  }
}

py::array_t<int> match_using_words(foundation::pyarray_f features1,
                                   foundation::pyarray_int words1,
                                   foundation::pyarray_f features2,
                                   foundation::pyarray_int words2,
                                   float lowes_ratio, int max_checks) {
  cv::Mat cv_f1 = foundation::pyarray_cv_mat_view(features1);
  cv::Mat cv_w1 = foundation::pyarray_cv_mat_view(words1);
  cv::Mat cv_f2 = foundation::pyarray_cv_mat_view(features2);
  cv::Mat cv_w2 = foundation::pyarray_cv_mat_view(words2);
  cv::Mat matches;

  MatchUsingWords(cv_f1, cv_w1, cv_f2, cv_w2, lowes_ratio, max_checks,
                  &matches);

  return foundation::py_array_from_cvmat<int>(matches);
}

VecXf compute_vlad_descriptor(const MatXf &features,
                              const MatXf &vlad_centers) {
  const auto vlad_center_size = vlad_centers.cols();
  const auto vlad_center_count = vlad_centers.rows();

  if (vlad_center_count == 0 || vlad_center_size == 0) {
    throw std::runtime_error("Zero VLAD centers or zero length VLAD words.");
  }

  VecXf vlad_descriptor(vlad_center_count * vlad_center_size);
  vlad_descriptor.setZero();

  for (int i = 0; i < features.rows(); ++i) {
    const auto &feature = features.row(i);

    float best_distance = std::numeric_limits<float>::max();
    int best_center = -1;
    for (int j = 0; j < vlad_center_count; ++j) {
      const float squared_norm = (feature - vlad_centers.row(j)).squaredNorm();
      if (squared_norm < best_distance) {
        best_distance = squared_norm;
        best_center = j;
      }
    }

    vlad_descriptor.segment(best_center * vlad_center_size, vlad_center_size) +=
        (feature - vlad_centers.row(best_center));
  }
  return vlad_descriptor;
}

std::pair<std::vector<double>, std::vector<std::string>> compute_vlad_distances(
    const std::map<std::string, VecXf> &vlad_descriptors,
    const std::string &image, std::set<std::string> &other_images) {
  if (vlad_descriptors.find(image) == vlad_descriptors.end()) {
    return std::make_pair(std::vector<double>(), std::vector<std::string>());
  }

  std::vector<double> distances;
  std::vector<std::string> others;
  const auto &reference = vlad_descriptors.at(image);
  for (const auto &candidate : other_images) {
    if (candidate == image) {
      continue;
    }
    const auto find_candidate = vlad_descriptors.find(candidate);
    if (find_candidate == vlad_descriptors.end()) {
      continue;
    }
    distances.push_back((reference - find_candidate->second).norm());
    others.push_back(candidate);
  }
  return std::make_pair(distances, others);
}
}  // namespace features
