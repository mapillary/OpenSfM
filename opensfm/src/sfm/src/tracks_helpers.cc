#include <sfm/tracks_helpers.h>

#include <unordered_set>

#include "sfm/types.h"

namespace sfm {
namespace tracks_helpers {
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

void AddConnections(TracksManager& manager, const ShotId& shot_id,
                    const std::vector<TrackId>& connections) {
  Observation observation;
  for (const auto& connection : connections) {
    manager.AddObservation(shot_id, connection, observation);
  }
}

void RemoveConnections(TracksManager& manager, const ShotId& shot_id,
                       const std::vector<TrackId>& connections) {
  for (const auto& connection : connections) {
    manager.RemoveObservation(shot_id, connection);
  }
}
}  // namespace tracks_helpers
}  // namespace sfm
