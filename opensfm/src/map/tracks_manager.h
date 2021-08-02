#pragma once

#include <map/defines.h>
#include <map/observation.h>

#include <fstream>
#include <map>
#include <unordered_map>
#include <vector>

namespace map {
class TracksManager {
 public:
  void AddObservation(const ShotId& shot_id, const TrackId& track_id,
                      const Observation& observation);
  void RemoveObservation(const ShotId& shot_id, const TrackId& track_id);
  Observation GetObservation(const ShotId& shot, const TrackId& track) const;

  int NumShots() const;
  int NumTracks() const;
  std::vector<ShotId> GetShotIds() const;
  std::vector<TrackId> GetTrackIds() const;

  const std::unordered_map<TrackId, Observation>& GetShotObservations(
      const ShotId& shot) const;
  const std::unordered_map<ShotId, Observation>& GetTrackObservations(
      const TrackId& track) const;

  TracksManager ConstructSubTracksManager(
      const std::vector<TrackId>& tracks,
      const std::vector<ShotId>& shots) const;

  using KeyPointTuple = std::tuple<TrackId, Observation, Observation>;
  std::vector<KeyPointTuple> GetAllCommonObservations(
      const ShotId& shot1, const ShotId& shot2) const;

  using ShotPair = std::pair<ShotId, ShotId>;
  std::unordered_map<ShotPair, int, HashPair> GetAllPairsConnectivity(
      const std::vector<ShotId>& shots,
      const std::vector<TrackId>& tracks) const;

  static TracksManager InstanciateFromFile(const std::string& filename);
  void WriteToFile(const std::string& filename) const;

  static TracksManager InstanciateFromString(const std::string& str);
  std::string AsSring() const;

  static TracksManager MergeTracksManager(
      const std::vector<const TracksManager*>& tracks_manager);

  bool HasShotObservations(const ShotId& shot) const;

  static std::string TRACKS_HEADER;
  static int TRACKS_VERSION;

 private:
  std::unordered_map<ShotId, std::unordered_map<TrackId, Observation>>
      tracks_per_shot_;
  std::unordered_map<TrackId, std::unordered_map<ShotId, Observation>>
      shots_per_track_;
};
}  // namespace map
