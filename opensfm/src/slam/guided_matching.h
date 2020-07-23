#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <map/shot.h>
#include <map/landmark.h>
#include <map/map.h>

namespace slam
{

struct GridParameters
{
  GridParameters(unsigned int grid_col_, unsigned int grid_rows,
                 float img_min_width, float img_min_height,
                 float img_max_width, float img_max_height,
                 float inv_cell_width, float inv_cell_height);
  const unsigned int grid_cols_, grid_rows_;
  const float img_min_width_, img_min_height_;
  const float img_max_width_, img_max_height_;
  const float inv_cell_width_, inv_cell_height_;


  template <class T>
  bool in_grid(const T x, const T y) const
  {
    return img_min_width_ < x && img_max_width_ > x && img_min_height_ < y && img_max_height_ > y;
  }
};

using CellIndices = std::vector<std::vector<std::vector<size_t>>>;
using MatchIndices = std::vector<std::pair<size_t, size_t>>;
class GuidedMatcher
{
public:
  static constexpr unsigned int HAMMING_DIST_THR_LOW{50};
  static constexpr unsigned int HAMMING_DIST_THR_HIGH{100};
  static constexpr unsigned int MAX_HAMMING_DIST{256};
  static constexpr float NO_LOWE_TEST{0.0f};
  static constexpr bool NO_ORIENTATION_CHECK{false};
  static constexpr size_t NO_MATCH{std::numeric_limits<size_t>::max()};


  static inline unsigned int
  compute_descriptor_distance_32(const DescriptorType &desc_1, const DescriptorType &desc_2)
  {
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

    constexpr uint32_t mask_1 = 0x55555555U;
    constexpr uint32_t mask_2 = 0x33333333U;
    constexpr uint32_t mask_3 = 0x0F0F0F0FU;
    constexpr uint32_t mask_4 = 0x01010101U;

    // const auto *pa = desc_1.ptr<uint32_t>();
    // const auto *pb = desc_2.ptr<uint32_t>();
    const uint32_t *pa = reinterpret_cast<const uint32_t*>(desc_1.data());
    const uint32_t *pb = reinterpret_cast<const uint32_t*>(desc_2.data());
    // const uint32_t *pa = desc_1.data();
    // const uint32_t *pb = desc_2.data();
    unsigned int dist = 0;

    for (unsigned int i = 0; i < 8; ++i, ++pa, ++pb)
    {
      auto v = *pa ^ *pb;
      v -= ((v >> 1) & mask_1);
      v = (v & mask_2) + ((v >> 2) & mask_2);
      dist += (((v + (v >> 4)) & mask_3) * mask_4) >> 24;
    }

    return dist;
  }

  const GridParameters &grid_params_;
  GuidedMatcher(const GridParameters &grid_params, const float scale_factor, const size_t num_scale_levels) : grid_params_(grid_params), num_scale_levels_(num_scale_levels), scale_factors_(num_scale_levels_, 1.0),
                                                                                                              scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor)), inv_level_sigma_sq_(num_scale_levels_, 1.0)
  {
    for (unsigned int level = 1; level < num_scale_levels_; ++level)
    {
      scale_factors_.at(level) = scale_factor * scale_factors_.at(level - 1);
      inv_level_sigma_sq_.at(level) = 1.0f / scale_factors_.at(level);
    }
  }

  void DistributeUndistKeyptsToGrid(const AlignedVector<Observation> &undist_keypts, CellIndices &keypt_indices_in_cells) const;

  MatchIndices MatchKptsToKpts(const AlignedVector<Observation> &undist_keypts_1, const DescriptorMatrix &descriptors_1,
                               const AlignedVector<Observation> &undist_keypts_2, const DescriptorMatrix &descriptors_2,
                               const CellIndices &keypts_indices_in_cells_2,
                               const Eigen::MatrixX2f &prevMatched, const size_t margin) const;
  // Matches the landmarks seen in last shot to the
  // keypts seen in curr shot and sets the observations
  // in curr shot accordingly
  size_t AssignShot1LandmarksToShot2Kpts(const map::Shot &last_shot, map::Shot &curr_shot, const float margin) const;

  size_t
  AssignLandmarksToShot(map::Shot &shot, const std::vector<map::Landmark *> &landmarks, const float margin,
                        const AlignedVector<Observation> &undist_kpts, bool check_orientation, const float lowe_ratio) const;
  map::FeatureId
  FindBestMatchForLandmark2(const map::Landmark *const landmark, map::Shot &curr_shot, const float reproj_x, const float reproj_y,
                            const int last_scale_level, const float margin) const;

  map::FeatureId
  FindBestMatchForLandmark(const map::Landmark *const lm, map::Shot &curr_shot,
                           const float reproj_x, const float reproj_y,
                           const int scale_level, const float margin,
                           const float lowe_ratio) const;
  std::vector<size_t>
  GetKeypointsInCell(const AlignedVector<Observation> &undist_keypts,
                     const CellIndices &keypt_indices_in_cells,
                     const float ref_x, const float ref_y, const float margin,
                     const int min_level = -1, const int max_level = -1) const;

  static size_t ComputeMedianDescriptorIdx(const AlignedVector<DescriptorType> &descriptors);

  bool
  IsObservable(map::Landmark *lm, const map::Shot &frame, const float ray_cos_thr,
               Eigen::Vector2d &reproj, size_t &pred_scale_level) const;

  size_t PredScaleLevel(const float max_valid_dist, const float cam_to_lm_dist) const;

  MatchIndices
  MatchingForTriangulationEpipolar(const map::Shot &kf1, const map::Shot &kf2, const Eigen::Matrix3d &E_12, const float min_depth, const float max_depth, const bool traverse_with_depth, const float margin = 5) const;
  
  static bool
  CheckEpipolarConstraint(const Eigen::Vector3d &bearing_1, const Eigen::Vector3d &bearing_2,
                          const Eigen::Matrix3d &E_12, const float bearing_1_scale_factor);

  template <typename T>
  size_t ReplaceDuplicatedLandmarks(map::Shot &fuse_shot, const T &landmarks_to_check, const float margin, map::Map &slam_map) const;

private:
  size_t num_scale_levels_;
  std::vector<float> scale_factors_;
  float scale_factor_;
  float log_scale_factor_;
  std::vector<float> inv_level_sigma_sq_;
};
}; // namespace slam