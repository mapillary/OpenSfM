#include <sfm/tracks_helpers.h>

#include <unordered_map>
#include <unordered_set>

namespace sfm {
namespace tracks_helpers {
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
}  // namespace tracks_helpers
}  // namespace sfm
