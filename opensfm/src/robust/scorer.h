#pragma once

#include <algorithm>
#include <vector>

template <class MODEL, class LOMODEL = MODEL>
struct ScoreInfo {
  double score{0};
  std::vector<int> inliers_indices;
  MODEL model;
  LOMODEL lo_model;

  friend bool operator<(const ScoreInfo& s1, const ScoreInfo& s2) {
    if (s1.score < s2.score) {
      return true;
    }
    return false;
  }
};

class RansacScoring {
 public:
  RansacScoring(double threshold) : threshold_(threshold) {}

  template <class IT, class T>
  ScoreInfo<T> Score(IT begin, IT end, const ScoreInfo<T>& best_score) const {
    ScoreInfo<T> score;
    for (IT it = begin; it != end; ++it) {
      if (it->norm() < threshold_) {
        ++score.score;
        score.inliers_indices.push_back(int(it - begin));
      }
    }
    return score;
  }
  double threshold_{0};
};

class MedianBasedScoring {
 public:
  MedianBasedScoring() = default;
  MedianBasedScoring(double nth) : nth_(nth) {}

  template <class IT>
  double ComputeMedian(IT begin, IT end) const {
    const int count = (end - begin);
    const int median_index = count * nth_;
    std::vector<double> norms(count);
    for (IT it = begin; it != end; ++it) {
      norms[(it - begin)] = it->norm();
    }
    std::nth_element(norms.begin(), norms.begin() + median_index, norms.end());
    return norms[median_index];
  }

 protected:
  double nth_{0.5};
};

class MSacScoring {
 public:
  MSacScoring(double threshold) : threshold_(threshold) {}

  template <class IT, class T>
  ScoreInfo<T> Score(IT begin, IT end, const ScoreInfo<T>& best_score) const {
    ScoreInfo<T> score;
    for (IT it = begin; it != end; ++it) {
      const auto v = (*it).norm();
      const int index = int(it - begin);
      if (v <= threshold_) {
        score.score += v * v;
        score.inliers_indices.push_back(index);
      } else {
        score.score += threshold_ * threshold_;
      }
    }
    const double eps = 1e-8;
    score.score = 1.0 / (score.score + eps);
    return score;
  }
  double threshold_{0};
};

class LMedSScoring : public MedianBasedScoring {
 public:
  LMedSScoring(double multiplier)
      : MedianBasedScoring(0.5), multiplier_(multiplier) {}

  template <class IT, class T>
  ScoreInfo<T> Score(IT begin, IT end, const ScoreInfo<T>& best_score) const {
    const auto median = this->ComputeMedian(begin, end);
    const auto mad = 1.4826 * median;
    const auto threshold = this->multiplier_ * mad;
    ScoreInfo<T> score;
    for (IT it = begin; it != end; ++it) {
      const auto v = it->norm();
      if (v <= threshold) {
        score.inliers_indices.push_back(int(it - begin));
      }
    }
    const double eps = 1e-8;
    score.score = 1.0 / (median + eps);
    return score;
  }

  double multiplier_;
};
