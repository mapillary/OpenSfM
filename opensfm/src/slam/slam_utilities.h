#pragma once
#include <Eigen/Core>
#include <vector>
#include <sfm/tracks_manager.h>
#include <slam/guided_matching.h>
#include <unordered_set>

namespace map
{
  class Shot;
  class Landmark;
}

namespace slam
{
class SlamUtilities
{
public:
  static Mat3d to_skew_symmetric_mat(const Vec3d& vec);

  static Mat3d create_E_21(const Mat3d& rot_1w, const Vec3d& trans_1w,
                           const Mat3d& rot_2w, const Vec3d& trans_2w);

  static MatXd ConvertOpenCVKptsToEigen(const AlignedVector<Observation>& keypts);
  

  static std::vector<map::Landmark*> update_local_landmarks(const std::vector<map::Shot*>& local_keyframes); //, const size_t curr_frm_id);

  static std::vector<map::Shot*> update_local_keyframes(const map::Shot& curr_shot);

  static size_t MatchShotToLocalMap(map::Shot &curr_shot, const slam::GuidedMatcher& matcher);

  static void SetDescriptorFromObservations(map::Landmark& landmark);
  static void SetDescriptorFromObservationsEig(map::Landmark& landmark);
  static void SetNormalAndDepthFromObservations(map::Landmark& landmark, const std::vector<float>& scale_factors);

  static std::pair<double, double> ComputeMinMaxDepthInShot(const map::Shot& shot);

  static void FuseDuplicatedLandmarks(map::Shot& shot,
                                      const std::vector<map::Shot*>& fuse_shots,
                                      const slam::GuidedMatcher& matcher,
                                      const float margin, map::Map& slam_map);

  
  static std::vector<map::Shot*> GetSecondOrderCovisibilityForShot(
      const map::Shot& shot, const size_t first_order_thr,
      const size_t second_order_thr);

  static std::unordered_map<map::ShotId, map::Shot*> ComputeLocalKeyframes(
      map::Shot& shot);
  static void TriangulateShotFeatures(const TracksManager& tracks_manager,
                                      map::Map& map, map::Shot* shot,
                                      const double reproj_threshold,
                                      const double min_reproj_angle);
  static void Triangulate(const TrackId lm_id,
                          const TracksManager& tracks_manager, map::Map& map,
                          const double reproj_threshold,
                          const double min_reproj_angle);
  static void Retriangulate(const TracksManager& tracks_manager, map::Map& map, const double reproj_threshold, const double min_reproj_angle,
                            const bool full_triangulation = true);
};
} // namespace map