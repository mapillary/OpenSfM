#pragma once

#include <sfm/observation.h>
#include <sfm/types.h>

#include <fstream>
#include <map>
#include <unordered_map>
#include <vector>

class TracksManager {
 public:
  void AddTrack(const TrackId& id,
                const std::unordered_map<ShotId, Observation>& track);
  void AddObservation(const ShotId& shot_id, const TrackId& track_id,
                      const Observation& observation);
  Observation GetObservation(const ShotId& shot, const TrackId& point) const;

  std::vector<ShotId> GetShotIds() const;
  std::vector<TrackId> GetTrackIds() const;

  std::unordered_map<TrackId, Observation> GetObservationsOfShot(
      const ShotId& shot) const;
  std::unordered_map<ShotId, Observation> GetObservationsOfPoint(
      const TrackId& point) const;
  std::unordered_map<TrackId, Observation> GetObservationsOfPointsAtShot(
      const std::vector<TrackId>& points, const ShotId& shot) const;

  using KeyPointTuple = std::tuple<TrackId, Observation, Observation>;
  std::vector<KeyPointTuple> GetAllCommonObservations(
      const ShotId& shot1, const ShotId& shot2) const;

  using ShotPair = std::pair<ShotId, ShotId>;
  std::unordered_map<ShotPair, std::vector<KeyPointTuple>, HashPair>
  GetAllCommonObservationsAllPairs() const;

  static TracksManager InstanciateFromFile(const std::string& filename);
  bool WriteToFile(const std::string& filename);

  static std::string TRACKS_HEADER;
  static int TRACKS_VERSION;

 private:
  std::unordered_map<ShotId, std::unordered_map<TrackId, Observation>>
      tracks_per_shot_;
  std::unordered_map<TrackId, std::unordered_map<ShotId, Observation>>
      shot_per_tracks_;
};