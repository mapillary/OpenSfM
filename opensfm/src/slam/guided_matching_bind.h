#pragma once
#include <slam/guided_matching.h>
#include <iostream>
namespace slam
{
class GuidedMatchingWrapper
{
public:
  GuidedMatchingWrapper(const GridParameters& grid_params, const float scale_factor, const size_t num_scale_levels):
    matcher_(grid_params, scale_factor, num_scale_levels) {}
  void DistributeUndistKeyptsToGrid(map::Shot& shot)
  {
    matcher_.DistributeUndistKeyptsToGrid(shot.slam_data_.undist_keypts_, shot.slam_data_.keypt_indices_in_cells_);
  }

  MatchIndices
  MatchShotToShot(map::Shot& shot1, map::Shot& shot2, const Eigen::MatrixX2f& prev_matched, const size_t margin) const
  {
    std::cout << "kpts:" << shot1.slam_data_.undist_keypts_.size() << "/" << shot2.slam_data_.undist_keypts_.size()
              << "cells: " << shot2.slam_data_.keypt_indices_in_cells_.size();
              //  << ", prev: " << prev_matched << "margin" << margin << std::endl;
    return matcher_.MatchKptsToKpts(shot1.slam_data_.undist_keypts_, shot1.GetDescriptors(),
                                    shot2.slam_data_.undist_keypts_, shot2.GetDescriptors(),
                                    shot2.slam_data_.keypt_indices_in_cells_,
                                    prev_matched, margin);
  }

  // size_t
  // AssignShot1LandmarksToShot2Kpts(const map::Shot& shot1, map::Shot& shot2, const float margin) const
  // {
  //   return matcher_.AssignShot1LandmarksToShot2Kpts(shot1, shot2, margin);
  // }

  size_t
  AssignShot1LandmarksToShot2KptsLM(const map::Shot& shot1, map::Shot& shot2, const float margin) const
  {
    const auto& lms = shot1.GetLandmarks();
    return matcher_.AssignLandmarksToShot(shot2, lms, margin, shot1.slam_data_.undist_keypts_,true, GuidedMatcher::NO_LOWE_TEST);
  }

  MatchIndices
  MatchForTriangulationEpipolar(const map::Shot& kf1, const map::Shot& kf2, const Eigen::Matrix3d& E_12, const float min_depth, const float max_depth, const bool traverse_with_depth, const float margin) const
  {
    return matcher_.MatchingForTriangulationEpipolar(kf1, kf2, E_12, min_depth, max_depth, traverse_with_depth, margin);
  }

// private:
  GuidedMatcher matcher_;
};
} // namespace slam
