#include <sfm/tracks_manager.h>

#include <sstream>
#include <unordered_set>

namespace {

template <class S>
int GetTracksFileVersion(S& fstream) {
  const auto current_position = fstream.tellg();

  std::string line;
  std::getline(fstream, line);

  int version = 0;
  if (line.find(TracksManager::TRACKS_HEADER) == 0) {
    version = std::atoi(
        line.substr(TracksManager::TRACKS_HEADER.length() + 2).c_str());
  } else {
    fstream.seekg(current_position);
  }
  return version;
}

template <class S>
void WriteToStreamCurrentVersion(S& ostream, const TracksManager& manager) {
  ostream << manager.TRACKS_HEADER << "_v" << manager.TRACKS_VERSION
          << std::endl;
  const auto shotsIDs = manager.GetShotIds();
  for (const auto& shotID : shotsIDs) {
    const auto observations = manager.GetShotObservations(shotID);
    for (const auto& observation : observations) {
      ostream << shotID << "\t" << observation.first << "\t"
              << observation.second.id << "\t" << observation.second.point(0)
              << "\t" << observation.second.point(1) << "\t"
              << observation.second.scale << "\t" << observation.second.color(0)
              << "\t" << observation.second.color(1) << "\t"
              << observation.second.color(2) << std::endl;
    }
  }
}

Observation InstanciateObservation(double x, double y, double scale, int id,
                                   int r, int g, int b) {
  Observation observation;
  observation.point << x, y;
  observation.scale = scale;
  observation.id = id;
  observation.color << r, g, b;
  return observation;
}

template <class S>
TracksManager InstanciateFromStreamV0(S& fstream) {
  ShotId image = "";
  TrackId trackID = "";
  int featureID = -1;
  double x = -1.0, y = -1.0;
  int r = 0, g = 0, b = 0;

  TracksManager manager;
  while (fstream >> image >> trackID >> featureID >> x >> y >> r >> g >> b) {
    auto observation = InstanciateObservation(x, y, 0., featureID, r, g, b);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
TracksManager InstanciateFromStreamV1(S& fstream) {
  ShotId image = "";
  TrackId trackID = "";
  int featureID = -1;
  double x = -1.0, y = -1.0, scale = 0.;
  int r = 0, g = 0, b = 0;

  TracksManager manager;
  while (fstream >> image >> trackID >> featureID >> x >> y >> scale >> r >>
         g >> b) {
    auto observation = InstanciateObservation(x, y, scale, featureID, r, g, b);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
TracksManager InstanciateFromStreamT(S& fstream) {
  const auto version = GetTracksFileVersion(fstream);
  switch (version) {
    case 0:
      return InstanciateFromStreamV0(fstream);
    case 1:
      return InstanciateFromStreamV1(fstream);
    default:
      throw std::runtime_error("Unknown tracks manager file version");
  }
}

}  // namespace

void TracksManager::AddObservation(const ShotId& shot_id,
                                   const TrackId& track_id,
                                   const Observation& observation) {
  tracks_per_shot_[shot_id][track_id] = observation;
  shots_per_track_[track_id][shot_id] = observation;
}

void TracksManager::RemoveObservation(const ShotId& shot_id,
                                      const TrackId& track_id) {
  const auto find_shot = tracks_per_shot_.find(shot_id);
  if (find_shot == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }
  const auto find_track = shots_per_track_.find(track_id);
  if (find_track == shots_per_track_.end()) {
    throw std::runtime_error("Accessing invalid track ID");
  }
  find_shot->second.erase(track_id);
  find_track->second.erase(shot_id);
}

int TracksManager::NumShots() const { return tracks_per_shot_.size(); }

int TracksManager::NumTracks() const { return shots_per_track_.size(); }

bool TracksManager::HasShotObservations(const ShotId& shot) const { return tracks_per_shot_.count(shot) > 0; }

std::vector<ShotId> TracksManager::GetShotIds() const {
  std::vector<ShotId> shots;
  shots.reserve(tracks_per_shot_.size());
  for (const auto& it : tracks_per_shot_) {
    shots.push_back(it.first);
  }
  return shots;
}

std::vector<TrackId> TracksManager::GetTrackIds() const {
  std::vector<TrackId> tracks;
  tracks.reserve(shots_per_track_.size());
  for (const auto& it : shots_per_track_) {
    tracks.push_back(it.first);
  }
  return tracks;
}

Observation TracksManager::GetObservation(const ShotId& shot,
                                          const TrackId& track) const {
  const auto find_shot = tracks_per_shot_.find(shot);
  if (find_shot == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }
  const auto find_track = find_shot->second.find(track);
  if (find_track == find_shot->second.end()) {
    throw std::runtime_error("Accessing invalid track ID");
  }
  return find_track->second;
}

const std::unordered_map<TrackId, Observation>&
TracksManager::GetShotObservations(const ShotId& shot) const {
  const auto find_shot = tracks_per_shot_.find(shot);
  if (find_shot == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }
  return find_shot->second;
}

const std::unordered_map<ShotId, Observation>&
TracksManager::GetTrackObservations(const TrackId& track) const {
  const auto find_track = shots_per_track_.find(track);
  if (find_track == shots_per_track_.end()) {
    throw std::runtime_error("Accessing invalid track ID");
  }
  return find_track->second;
}

TracksManager TracksManager::ConstructSubTracksManager(
    const std::vector<TrackId>& tracks,
    const std::vector<ShotId>& shots) const {
  std::unordered_set<TrackId> shotsTmp;
  for (const auto& id : shots) {
    shotsTmp.insert(id);
  }

  TracksManager subset;
  for (const auto& track_id : tracks) {
    const auto find_track = shots_per_track_.find(track_id);
    if (find_track == shots_per_track_.end()) {
      continue;
    }
    for (const auto& obs : find_track->second) {
      const auto& shot_id = obs.first;
      if (shotsTmp.find(shot_id) == shotsTmp.end()) {
        continue;
      }
      subset.AddObservation(shot_id, track_id, obs.second);
    }
  }
  return subset;
}

std::vector<TracksManager::KeyPointTuple>
TracksManager::GetAllCommonObservations(const ShotId& shot1,
                                        const ShotId& shot2) const {
  auto find_shot1 = tracks_per_shot_.find(shot1);
  auto find_shot2 = tracks_per_shot_.find(shot2);
  if (find_shot1 == tracks_per_shot_.end() ||
      find_shot2 == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }

  std::vector<KeyPointTuple> tuples;
  for (const auto& p : find_shot1->second) {
    const auto find = find_shot2->second.find(p.first);
    if (find != find_shot2->second.end()) {
      tuples.emplace_back(p.first, p.second, find->second);
    }
  }
  return tuples;
}

std::unordered_map<TracksManager::ShotPair, int, HashPair>
TracksManager::GetAllPairsConnectivity(
    const std::vector<ShotId>& shots,
    const std::vector<TrackId>& tracks) const {
  std::unordered_map<ShotPair, int, HashPair> common_per_pair;

  std::vector<TrackId> tracks_to_use;
  if (tracks.empty()) {
    for (const auto& track : shots_per_track_) {
      tracks_to_use.push_back(track.first);
    }
  } else {
    tracks_to_use = tracks;
  }

  std::unordered_set<ShotId> shots_to_use;
  if (shots.empty()) {
    for (const auto& shot : tracks_per_shot_) {
      shots_to_use.insert(shot.first);
    }
  } else {
    for (const auto& shot : shots) {
      shots_to_use.insert(shot);
    }
  }

  for (const auto& track_id : tracks_to_use) {
    const auto find_track = shots_per_track_.find(track_id);
    if (find_track == shots_per_track_.end()) {
      continue;
    }
    const auto& track = find_track->second;
    for (const auto& it1 : track) {
      const auto& shot_id1 = it1.first;
      if (shots_to_use.find(shot_id1) != shots_to_use.end()) {
        for (const auto& it2 : track) {
          const auto& shot_id2 = it2.first;
          if (shot_id1 < shot_id2 &&
              shots_to_use.find(shot_id2) != shots_to_use.end()) {
            ++common_per_pair[std::make_pair(shot_id1, shot_id2)];
          }
        }
      }
    }
  }
  return common_per_pair;
}

TracksManager TracksManager::InstanciateFromFile(const std::string& filename) {
  std::ifstream istream(filename);
  if (istream.is_open()) {
    return InstanciateFromStreamT(istream);
  } else {
    throw std::runtime_error("Can't read tracks manager file");
  }
}

void TracksManager::WriteToFile(const std::string& filename) const {
  std::ofstream ostream(filename);
  if (ostream.is_open()) {
    WriteToStreamCurrentVersion(ostream, *this);
  } else {
    throw std::runtime_error("Can't write tracks manager file");
  }
}

TracksManager TracksManager::InstanciateFromString(const std::string& str) {
  std::stringstream sstream(str);
  return InstanciateFromStreamT(sstream);
}

std::string TracksManager::AsSring() const {
  std::stringstream sstream;
  WriteToStreamCurrentVersion(sstream, *this);
  return sstream.str();
}

std::string TracksManager::TRACKS_HEADER = "OPENSFM_TRACKS_VERSION";
int TracksManager::TRACKS_VERSION = 1;
