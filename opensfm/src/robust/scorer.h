#pragma once

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
  RansacScoring(double threshold);

  template <class IT, class T>
  ScoreInfo<T> Score(IT begin, IT end, const ScoreInfo<T>& best_score);
  double threshold_{0};
};

class MedianBasedScoring {
 public:
  MedianBasedScoring() = default;
  MedianBasedScoring(double nth);

  template <class IT>
  double ComputeMedian(IT begin, IT end);

 protected:
  double nth_{0.5};
};

class MSacScoring {
 public:
  MSacScoring(double threshold);

  template <class IT, class T>
  ScoreInfo<T> Score(IT begin, IT end, const ScoreInfo<T>& best_score);
  double threshold_{0};
};

class LMedSScoring : public MedianBasedScoring {
 public:
  LMedSScoring(double multiplier);

  template <class IT, class T>
  ScoreInfo<T> Score(IT begin, IT end, const ScoreInfo<T>& best_score);

  double multiplier_;
};