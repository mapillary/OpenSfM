#pragma once

#include <sfm/types.h>

#include <map>
#include <vector>
#include <fstream>
#include <unordered_map>

class TracksManager {
 public:

  void AddTrack(int id, const std::unordered_map<ShotId, cv::KeyPoint>& track){
      for(const auto observation : track){
          tracks_per_shot_[observation.first][id] = observation.second;
          shot_per_tracks_[id][observation.first] = observation.second;
      }   
  }
  std::vector<ShotId> GetShotIds(){
      std::vector<ShotId> shots;
      for(const auto& it : tracks_per_shot_){
          shots.push_back(it.first);
      }
      return shots;
  }

  std::vector<TrackId> GetTrackIds(){
      std::vector<TrackId> tracks;
      for(const auto& it : shot_per_tracks_){
          tracks.push_back(it.first);
      }
      return tracks;
  }

  cv::KeyPoint GetObservation(const ShotId& shot, const TrackId& point){
      const auto findShot = tracks_per_shot_.find(shot);
      if(findShot == tracks_per_shot_.end()){
          throw std::runtime_error("Accessing invalid shot ID");
      }
      const auto findPoint = findShot->second.find(point);
      if(findPoint == findShot->second.end()){
          throw std::runtime_error("Accessing invalid point ID");
      }
      return findPoint->second;
  }

  // Not sure if we use that
  std::unordered_map<TrackId, cv::KeyPoint> GetObservationsOfShot(const ShotId& shot){
      const auto findShot = tracks_per_shot_.find(shot);
      if(findShot == tracks_per_shot_.end()){
          throw std::runtime_error("Accessing invalid shot ID");
      }
      return findShot->second;
  }

  // For point triangulation
  std::unordered_map<ShotId, cv::KeyPoint> GetObservationsOfPoint(const TrackId& point){
      const auto findPoint = shot_per_tracks_.find(point);
      if(findPoint == shot_per_tracks_.end()){
          throw std::runtime_error("Accessing invalid point ID");
      }
      return findPoint->second;
  }

  // For shot resection
  std::unordered_map<TrackId, cv::KeyPoint> GetObservationsOfPointsAtShot(
      const std::vector<TrackId>& points, const ShotId& shot){
        std::unordered_map<TrackId, cv::KeyPoint> obervations;
        const auto findShot = tracks_per_shot_.find(shot);
        if(findShot == tracks_per_shot_.end()){
            throw std::runtime_error("Accessing invalid shot ID");
        }
        for(const auto point : points){
            const auto findPoint = findShot->second.find(point);
            if(findPoint == findShot->second.end()){
                continue;
            }
            obervations[point] = findPoint->second;
        }
        return obervations;
    }

  // For pair bootstrapping
  using KeyPointPair = std::pair<cv::KeyPoint, cv::KeyPoint>;
  std::vector<KeyPointPair> GetAllCommonObservations(
      const ShotId& shot1, const ShotId& shot2) {
    auto findShot1 = tracks_per_shot_.find(shot1);
    auto findShot2 = tracks_per_shot_.find(shot2);
    if (findShot1 == tracks_per_shot_.end() ||
        findShot2 == tracks_per_shot_.end()) {
      throw std::runtime_error("Accessing invalid shot ID");
    }

    std::unordered_map<TrackId, std::vector<cv::KeyPoint>> per_track;
    for(const auto& p : findShot1->second){
        per_track[p.first].push_back(p.second);
    }
    for(const auto& p : findShot2->second){
        per_track[p.first].push_back(p.second);
    }

    std::vector<KeyPointPair> pairs;
    for(const auto& p : per_track){
        if(p.second.size() < 2){
            continue;
        }
        pairs.push_back(std::make_pair(p.second[0], p.second[1]));
    }
    return pairs;
  }

  // I/O
  static TracksManager InstanciateFromFile(const std::string& filename) {
    std::ifstream istream(filename);
    if (istream.is_open()) {
      const auto version = GetTracksFileVersion(istream);
      switch (version) {
        case 0:
          return InstanciateFromFileV0(istream);
        case 1:
          return InstanciateFromFileV1(istream);
        default:
          throw std::runtime_error("Unknown tracks manager file version");
      }
    } else {
      throw std::runtime_error("Can't read tracks manager file");
    }
  }

  static bool WriteToFile(){

  }

  private :

  static std::string TRACKS_HEADER;
  static int TRACKS_VERSION;

  static int GetTracksFileVersion(std::ifstream& fstream) {
    const auto current_position = fstream.tellg();

    std::string line;
    std::getline(fstream, line);

    int version = 0;
    if (line.find(TRACKS_HEADER) == 0) {
        version = std::atoi(line.substr(TRACKS_HEADER.length()).c_str());
    } else {
      fstream.seekg(current_position);
    }
    return version;
  }

  static cv::KeyPoint InstanciateKeypoint(double x, double y, double scale, int id) {
    cv::KeyPoint keypoint;
    keypoint.pt.x = x;
    keypoint.pt.y = y;
    keypoint.size = scale;
    keypoint.class_id = id;
  }
  static TracksManager InstanciateFromFileV0(std::ifstream& fstream){
      std::string image = "", linw = "";
      int trackID = -1, featureID = -1;
      double x = -1.0, y = -1.0;
      char r = 0, g = 0, b = 0;

      TracksManager manager;
      while(fstream >> image >> trackID >> featureID >> x >> y >> r >> g >> b){
          auto keypoint = InstanciateKeypoint(x, y, 0., featureID);
          manager.tracks_per_shot_[image][trackID] = keypoint;
          manager.shot_per_tracks_[trackID][image] = keypoint;
      }

  }

  static TracksManager InstanciateFromFileV1(std::ifstream& fstream){
      std::string image = "", linw = "";
      int trackID = -1, featureID = -1;
      double x = -1.0, y = -1.0, scale = 0.;
      char r = 0, g = 0, b = 0;

      TracksManager manager;
      while(fstream >> image >> trackID >> featureID >> x >> y >> scale >> r >> g >> b){
          auto keypoint = InstanciateKeypoint(x, y, scale, featureID);
          manager.tracks_per_shot_[image][trackID] = keypoint;
          manager.shot_per_tracks_[trackID][image] = keypoint;
      }
  }

  std::unordered_map< ShotId, std::unordered_map<TrackId, cv::KeyPoint> > tracks_per_shot_;
  std::unordered_map< TrackId, std::unordered_map<ShotId, cv::KeyPoint> > shot_per_tracks_;
};

std::string TracksManager::TRACKS_HEADER = "OPENSFM_TRACKS_VERSION";
int TracksManager::TRACKS_VERSION = 1;