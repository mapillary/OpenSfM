#include <sfm/tracks_helpers.h>

#include <unordered_map>
#include <unordered_set>

namespace sfm::tracks_helpers {
std::unordered_map<map::ShotId, int> CountTracksPerShot(
    const map::TracksManager& manager, const std::vector<map::ShotId>& shots,
    const std::vector<map::TrackId>& tracks) {
  std::unordered_set<map::TrackId> tracks_set;
  for (const auto& track : tracks) {
    tracks_set.insert(track);
  }
  
  std::unordered_map<map::ShotId, int> counts;
  for (int i = 0; i < shots.size(); ++i) {
    counts[shots[i]] = 0;
  }

#pragma omp parallel
{
  std::vector<int> thread_counts(shots.size(), 0);
#pragma omp for
  for (int i = 0; i < shots.size(); ++i) {
    const auto& shot = shots[i];
    const auto& observations = manager.GetShotObservations(shot);

    int sum = 0;
    for (const auto& obs : observations) {
      const auto& trackID = obs.first;
      if (tracks_set.find(trackID) == tracks_set.end()) {
        continue;
      }
      ++sum;
    }
    thread_counts[i] = sum;
  }

#pragma omp critical
  {
    for (int i = 0; i < shots.size(); ++i) {
      if (thread_counts[i] > 0) {
        counts[shots[i]] = thread_counts[i];
      }
    }
  }
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
}  // namespace sfm::tracks_helpers
