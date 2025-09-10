#include "sfm/tracks_helpers.h"

#include <geometry/covariance.h>
#include <geometry/triangulation.h>
#include <map/landmark.h>
#include <map/shot.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_set>
#include <vector>

extern "C" {
#include <vl/kdtree.h>
}

namespace sfm::tracks_helpers {
std::unordered_map<map::ShotId, int> CountTracksPerShot(
    const map::TracksManager& manager, const std::vector<map::ShotId>& shots,
    const std::vector<map::TrackId>& tracks) {
  std::unordered_set<map::TrackId> tracks_set;
  for (const auto& track : tracks) {
    tracks_set.insert(track);
  }
  std::unordered_map<map::ShotId, int> counts;
  for (const auto& shot : shots) {
    const auto& observations = manager.GetShotObservations(shot);

    int sum = 0;
    for (const auto& obs : observations) {
      const auto& trackID = obs.first;
      if (tracks_set.find(trackID) == tracks_set.end()) {
        continue;
      }
      ++sum;
    }
    counts[shot] = sum;
  }
  return counts;
}

void AddConnections(map::TracksManager& manager, const map::ShotId& shot_id,
                    const std::vector<map::TrackId>& connections) {
  map::Observation observation;
  for (const auto& connection : connections) {
    manager.AddObservation(shot_id, connection, observation);
  }
}

void RemoveConnections(map::TracksManager& manager, const map::ShotId& shot_id,
                       const std::vector<map::TrackId>& connections) {
  for (const auto& connection : connections) {
    manager.RemoveObservation(shot_id, connection);
  }
}

int FilterBadlyConditionedPoints(map::Map& map, double min_angle_deg,
                                 double min_abs_det) {
  std::vector<map::LandmarkId> to_remove;

  // Cull based on stats on covariance's condition number
  constexpr double kMaxCond = 1000;
  constexpr double kSigmaMultiplier = 1.0;
  std::vector<std::pair<map::LandmarkId, double>> landmark_conds;
  landmark_conds.reserve(map.GetLandmarks().size());

  // Iterate over a snapshot of landmarks to decide removals.
  for (const auto& kv : map.GetLandmarks()) {
    const map::LandmarkId& lm_id = kv.first;
    const map::Landmark& lm = kv.second;

    const auto& observations = lm.GetObservations();

    std::vector<geometry::Camera> cameras;
    cameras.reserve(observations.size());
    std::vector<geometry::Pose> poses;
    poses.reserve(observations.size());
    std::vector<Vec2d> obs_vec;
    obs_vec.reserve(observations.size());

    // Build camera/pose/observation vectors from observations.
    for (const auto& obs_kv : observations) {
      map::Shot* shot_ptr = obs_kv.first;
      if (!shot_ptr) {
        continue;
      }

      const geometry::Pose* pose_ptr = shot_ptr->GetPose();
      const geometry::Camera* cam_ptr = shot_ptr->GetCamera();
      if (!pose_ptr || !cam_ptr) {
        continue;
      }

      // Get the shot's Observation for this landmark.
      map::Observation* obs_ptr =
          shot_ptr->GetLandmarkObservation(const_cast<map::Landmark*>(&lm));
      if (!obs_ptr) {
        continue;
      }

      cameras.push_back(*cam_ptr);
      poses.push_back(*pose_ptr);
      obs_vec.push_back(obs_ptr->point);
    }

    // First simple check based on angle between all pairs of raysmap
    double rad_angle = min_angle_deg * M_PI / 180.0;
    bool to_keep = false;
    for (size_t i = 0; i < poses.size() && !to_keep; ++i) {
      for (size_t j = i + 1; j < poses.size() && !to_keep; ++j) {
        const auto ray1 =
            (lm.GetGlobalPos() - poses[i].GetOrigin()).normalized();
        const auto ray2 =
            (lm.GetGlobalPos() - poses[j].GetOrigin()).normalized();
        const double angle = geometry::AngleBetweenVectors(ray1, ray2);
        if (angle > rad_angle) {
          to_keep = true;
        }
      }
    }
    if (!to_keep) {
      to_remove.push_back(lm_id);
      continue;
    }

    // Compute inverse covariance for the point.
    auto invcov_and_score = geometry::covariance::ComputePointInverseCovariance(
        cameras, poses, obs_vec, lm.GetGlobalPos());
    const auto& invcov = invcov_and_score.first;

    // Basic numerical checks.
    if (!invcov.allFinite()) {
      to_remove.push_back(lm_id);
      continue;
    }

    // Determinant-based test.
    const double det = invcov.determinant();
    if (!std::isfinite(det) || std::abs(det) < min_abs_det) {
      to_remove.push_back(lm_id);
      continue;
    }

    // Eigenvalue-based test (condition number).
    Eigen::SelfAdjointEigenSolver<Mat3d> es(invcov.inverse());
    if (es.info() != Eigen::Success) {
      to_remove.push_back(lm_id);
      continue;
    }
    const auto& eigs = es.eigenvalues();
    if (!(eigs.array().isFinite().all())) {
      to_remove.push_back(lm_id);
      continue;
    }

    const double max_eig = eigs.maxCoeff();
    const double min_eig = eigs.minCoeff();

    // If any eigenvalue is non-positive or extremely small, mark removal.
    if (min_eig <= 0.0 || !std::isfinite(min_eig)) {
      to_remove.push_back(lm_id);
      continue;
    }

    const double cond = std::min(std::sqrt(max_eig / min_eig), kMaxCond);
    if (!std::isfinite(cond)) {
      to_remove.push_back(lm_id);
      continue;
    }
    landmark_conds.emplace_back(lm_id, cond);
  }

  // Compute median and MAD of condition numbers.
  if (!landmark_conds.empty()) {
    const double avg_cond =
        std::accumulate(
            landmark_conds.begin(), landmark_conds.end(), 0.0,
            [](double sum, const auto& pair) { return sum + pair.second; }) /
        landmark_conds.size();
    const double sigma = std::sqrt(
        std::accumulate(landmark_conds.begin(), landmark_conds.end(), 0.0,
                        [avg_cond](double sum, const auto& pair) {
                          return sum + (pair.second - avg_cond) *
                                           (pair.second - avg_cond);
                        }) /
        landmark_conds.size());
    const double threshold = avg_cond + kSigmaMultiplier * sigma;
    for (const auto& [lm_id, cond] : landmark_conds) {
      if (cond > threshold) {
        to_remove.push_back(lm_id);
      }
    }
  }

  // Remove collected landmarks from the map.
  int removed = 0;
  for (const auto& lm_id : to_remove) {
    if (map.HasLandmark(lm_id)) {
      map.RemoveLandmark(lm_id);
      ++removed;
    }
  }

  return removed;
}

int RemoveIsolatedPoints(map::Map& map, int k) {
  const auto& landmarks = map.GetLandmarks();
  if (landmarks.size() <= k) {
    return 0;
  }

  // Gather all positions
  std::vector<map::LandmarkId> lm_ids;
  std::vector<float> positions;  // flattened for VLFeat (x0,y0,z0,x1,y1,z1,...)
  lm_ids.reserve(landmarks.size());
  positions.reserve(landmarks.size() * 3);
  for (const auto& kv : landmarks) {
    lm_ids.push_back(kv.first);
    const Vec3d& pos = kv.second.GetGlobalPos();
    positions.push_back(static_cast<float>(pos.x()));
    positions.push_back(static_cast<float>(pos.y()));
    positions.push_back(static_cast<float>(pos.z()));
  }

  // Build VLFeat KD-tree
  VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 3, 1, VlDistanceL2);
  vl_kdforest_build(forest, lm_ids.size(), positions.data());

  // Query kNN for each point in parallel
  std::vector<double> avg_dists(lm_ids.size(), 0.0);
  std::vector<VlKDForestNeighbor> neighbors(k + 1);
  for (vl_size i = 0; i < lm_ids.size(); ++i) {
    const float* query = positions.data() + 3 * i;
    vl_kdforest_query(forest, neighbors.data(), k + 1, query);

    // neighbors[0] is always the query point itself (distance 0)
    double sum = 0.0;
    int found = 0;
    for (int j = 1; j < k + 1; ++j) {
      if (std::isfinite(neighbors[j].distance)) {
        sum += neighbors[j].distance;
        ++found;
      }
    }
    avg_dists[i] = found > 0 ? sum / found : 0.0;
  }
  vl_kdforest_delete(forest);

  // Compute mean and stddev of average neighbor distances
  const double mean = std::accumulate(avg_dists.begin(), avg_dists.end(), 0.0) /
                      avg_dists.size();
  const double std_dev =
      std::sqrt(std::accumulate(avg_dists.begin(), avg_dists.end(), 0.0,
                                [mean](double sum, double val) {
                                  return sum + (val - mean) * (val - mean);
                                }) /
                avg_dists.size());

  // Remove points whose avg kNN distance > mean + stddev
  constexpr double kSigmaMultiplier = 1.25;
  int removed = 0;
  for (size_t i = 0; i < lm_ids.size(); ++i) {
    if (avg_dists[i] > mean + kSigmaMultiplier * std_dev) {
      if (map.HasLandmark(lm_ids[i])) {
        map.RemoveLandmark(lm_ids[i]);
        ++removed;
      }
    }
  }
  return removed;
}

}  // namespace sfm::tracks_helpers
