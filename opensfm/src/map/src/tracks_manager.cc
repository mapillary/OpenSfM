#include <foundation/union_find.h>
#include <map/tracks_manager.h>

#include <optional>
#include <sstream>
#include <unordered_set>

namespace {

template <class S>
int GetTracksFileVersion(S& fstream) {
  const auto current_position = fstream.tellg();

  std::string line;
  std::getline(fstream, line);

  int version = 0;
  if (line.find(map::TracksManager::TRACKS_HEADER) == 0) {
    version = std::atoi(
        line.substr(map::TracksManager::TRACKS_HEADER.length() + 2).c_str());
  } else {
    fstream.seekg(current_position);
  }
  return version;
}

template <class S>
void WriteToStreamCurrentVersion(S& ostream,
                                 const map::TracksManager& manager) {
  ostream << manager.TRACKS_HEADER << "_v" << manager.TRACKS_VERSION
          << std::endl;
  const auto shotsIDs = manager.GetShotIds();
  for (const auto& shotID : shotsIDs) {
    const auto observations = manager.GetShotObservations(shotID);
    for (const auto& observation : observations) {
      ostream << shotID << "\t" << observation.first << "\t"
              << observation.second.feature_id << "\t"
              << observation.second.point(0) << "\t"
              << observation.second.point(1) << "\t" << observation.second.scale
              << "\t" << observation.second.color(0) << "\t"
              << observation.second.color(1) << "\t"
              << observation.second.color(2) << "\t"
              << observation.second.segmentation_id << "\t"
              << observation.second.instance_id << std::endl;
    }
  }
}

map::Observation InstanciateObservation(
    double x, double y, double scale, int id, int r, int g, int b,
    int segm = map::Observation::NO_SEMANTIC_VALUE,
    int inst = map::Observation::NO_SEMANTIC_VALUE) {
  map::Observation observation;
  observation.point << x, y;
  observation.scale = scale;
  observation.feature_id = id;
  observation.color << r, g, b;
  observation.segmentation_id = segm;
  observation.instance_id = inst;
  return observation;
}

void SeparateLineByTabs(const std::string& line,
                        std::vector<std::string>& elems) {
  elems.clear();
  std::stringstream stst(line);
  std::string elem;
  while (std::getline(stst, elem, '\t'))  // separate by tabs
  {
    elems.push_back(elem);
  }
}

template <class S>
map::TracksManager InstanciateFromStreamV0(S& fstream) {
  map::TracksManager manager;
  std::string line;
  std::vector<std::string> elems;
  constexpr auto N_ENTRIES{8};
  elems.reserve(N_ENTRIES);
  while (std::getline(fstream, line)) {
    SeparateLineByTabs(line, elems);
    if (elems.size() != N_ENTRIES)  // process only valid lines
    {
      std::runtime_error(
          "Encountered invalid line. A line must contain exactly " +
          std::to_string(N_ENTRIES) + " values!");
    }
    const map::ShotId image = elems[0];
    const map::TrackId trackID = elems[1];
    const int featureID = std::stoi(elems[2]);
    const double x = std::stod(elems[3]);
    const double y = std::stod(elems[4]);
    const double scale = 0.0;
    const int r = std::stoi(elems[5]);
    const int g = std::stoi(elems[6]);
    const int b = std::stoi(elems[7]);
    auto observation = InstanciateObservation(x, y, scale, featureID, r, g, b);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
map::TracksManager InstanciateFromStreamV1(S& fstream) {
  map::TracksManager manager;
  std::string line;
  std::vector<std::string> elems;
  constexpr auto N_ENTRIES{9};
  elems.reserve(N_ENTRIES);
  while (std::getline(fstream, line)) {
    SeparateLineByTabs(line, elems);
    if (elems.size() != N_ENTRIES)  // process only valid lines
    {
      std::runtime_error(
          "Encountered invalid line. A line must contain exactly " +
          std::to_string(N_ENTRIES) + " values!");
    }
    const map::ShotId image = elems[0];
    const map::TrackId trackID = elems[1];
    const int featureID = std::stoi(elems[2]);
    const double x = std::stod(elems[3]);
    const double y = std::stod(elems[4]);
    const double scale = std::stod(elems[5]);
    const int r = std::stoi(elems[6]);
    const int g = std::stoi(elems[7]);
    const int b = std::stoi(elems[8]);
    auto observation = InstanciateObservation(x, y, scale, featureID, r, g, b);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
map::TracksManager InstanciateFromStreamV2(S& fstream) {
  map::TracksManager manager;
  std::string line;
  std::vector<std::string> elems;
  constexpr auto N_ENTRIES{11};
  elems.reserve(N_ENTRIES);
  while (std::getline(fstream, line)) {
    SeparateLineByTabs(line, elems);
    if (elems.size() != N_ENTRIES)  // process only valid lines
    {
      std::runtime_error(
          "Encountered invalid line. A line must contain exactly " +
          std::to_string(N_ENTRIES) + " values!");
    }
    const map::ShotId image = elems[0];
    const map::TrackId trackID = elems[1];
    const int featureID = std::stoi(elems[2]);
    const double x = std::stod(elems[3]);
    const double y = std::stod(elems[4]);
    const double scale = std::stod(elems[5]);
    const int r = std::stoi(elems[6]);
    const int g = std::stoi(elems[7]);
    const int b = std::stoi(elems[8]);
    const int segm = std::stoi(elems[9]);
    const int inst = std::stoi(elems[10]);
    auto observation =
        InstanciateObservation(x, y, scale, featureID, r, g, b, segm, inst);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
map::TracksManager InstanciateFromStreamT(S& fstream) {
  const auto version = GetTracksFileVersion(fstream);
  switch (version) {
    case 0:
      return InstanciateFromStreamV0(fstream);
    case 1:
      return InstanciateFromStreamV1(fstream);
    case 2:
      return InstanciateFromStreamV2(fstream);
    default:
      throw std::runtime_error("Unknown tracks manager file version");
  }
}

}  // namespace

namespace map {
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

bool TracksManager::HasShotObservations(const ShotId& shot) const {
  return tracks_per_shot_.count(shot) > 0;
}

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

TracksManager TracksManager::MergeTracksManager(
    const std::vector<const TracksManager*>& tracks_managers) {
  // Some typedefs claryfying the aggregations
  using FeatureId = std::pair<ShotId, int>;
  using SingleTrackId = std::pair<TrackId, int>;

  // Union-find main data
  std::vector<std::unique_ptr<UnionFindElement<SingleTrackId>>>
      union_find_elements;

  // Aggregate tracks be merged using (shot_id, feature_id)
  std::unordered_map<FeatureId, std::vector<int>, HashPair>
      observations_per_feature_id;
  for (int i = 0; i < tracks_managers.size(); ++i) {
    const auto& manager = tracks_managers[i];
    for (const auto& track_obses : manager->shots_per_track_) {
      const auto element_id = union_find_elements.size();
      for (const auto& shot_obs : track_obses.second) {
        observations_per_feature_id[std::make_pair(shot_obs.first,
                                                   shot_obs.second.feature_id)]
            .push_back(element_id);
      }
      union_find_elements.emplace_back(
          std::unique_ptr<UnionFindElement<SingleTrackId>>(
              new UnionFindElement<SingleTrackId>(
                  std::make_pair(track_obses.first, i))));
    }
  }

  TracksManager merged;
  if (union_find_elements.empty()) {
    return merged;
  }

  // Union-find any two tracks sharing a common FeatureId
  // For N tracks, make 0 the parent of [1, ... N-1[
  for (const auto& tracks_agg : observations_per_feature_id) {
    if (tracks_agg.second.empty()) {
      continue;
    }
    const auto e1 = union_find_elements[tracks_agg.second[0]].get();
    for (int i = 1; i < tracks_agg.second.size(); ++i) {
      const auto e2 = union_find_elements[tracks_agg.second[i]].get();
      Union(e1, e2);
    }
  }

  // Get clusters and construct new tracks
  const auto clusters = GetUnionFindClusters(&union_find_elements);
  for (int i = 0; i < clusters.size(); ++i) {
    const auto& tracks_agg = clusters[i];
    const auto merged_track_id = std::to_string(i);
    // Run over tracks to merged into a new single track
    for (const auto& manager_n_track_id : tracks_agg) {
      const auto manager_id = manager_n_track_id->data.second;
      const auto track_id = manager_n_track_id->data.first;
      const auto track =
          tracks_managers[manager_id]->shots_per_track_.at(track_id);
      for (const auto& shot_obs : track) {
        merged.AddObservation(shot_obs.first, merged_track_id, shot_obs.second);
      }
    }
  }
  return merged;
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
int TracksManager::TRACKS_VERSION = 2;
}  // namespace map
