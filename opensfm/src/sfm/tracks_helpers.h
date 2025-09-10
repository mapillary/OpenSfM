#pragma once

#include <foundation/types.h>
#include <geometry/camera.h>
#include <map/defines.h>
#include <map/map.h>
#include <map/observation.h>
#include <map/shot.h>
#include <map/tracks_manager.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace sfm::tracks_helpers {
std::unordered_map<map::ShotId, int> CountTracksPerShot(
    const map::TracksManager& manager, const std::vector<map::ShotId>& shots,
    const std::vector<map::TrackId>& tracks);
void AddConnections(map::TracksManager& manager, const map::ShotId& shot_id,
                    const std::vector<map::TrackId>& connections);
void RemoveConnections(map::TracksManager& manager, const map::ShotId& shot_id,
                       const std::vector<map::TrackId>& connections);

int FilterBadlyConditionedPoints(map::Map& map, double min_angle_deg = 1.0,
                                 double min_abs_det = 1e-15);
int RemoveIsolatedPoints(map::Map& map, int k = 7);

}  // namespace sfm::tracks_helpers
