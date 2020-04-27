#include <sfm/sfm_helpers.h>
#include <unordered_set>

namespace sfm_helpers {

std::unordered_map<ShotId, int> CountTracksPerShot(
    const TracksManager& manager, const std::vector<ShotId>& shots,
    const std::vector<TrackId>& tracks) {
  std::unordered_set<TrackId> tracks_set;
  for (const auto& track : tracks) {
    tracks_set.insert(track);
  }
  std::unordered_map<ShotId, int> counts;
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

std::unordered_map<ShotPair, int, HashPair> CountCommonObservations(
    const TracksManager& manager, const std::vector<ShotId>& shots,
    const std::vector<TrackId>& tracks) {
  const auto pairs = manager.GetAllCommonObservationsAllPairs(tracks);

  std::unordered_set<TrackId> shots_set;
  for (const auto& shot : shots) {
    shots_set.insert(shot);
  }

  std::unordered_map<ShotPair, int, HashPair> counts;
  for (const auto& pair : pairs) {
    if (shots_set.find(pair.first.first) == shots_set.end() ||
        shots_set.find(pair.first.second) == shots_set.end()) {
      continue;
    }
    counts[pair.first] = pair.second.size();
  }
  return counts;
}
}  // namespace sfm_helpers