#pragma once

#include <foundation/types.h>
#include <geometry/camera.h>
#include <map/defines.h>
#include <map/observation.h>
#include <map/shot.h>
#include <map/tracks_manager.h>

#include <string>

namespace sfm {
namespace tracks_helpers {
std::unordered_map<map::ShotId, int> CountTracksPerShot(
    const map::TracksManager& manager, const std::vector<map::ShotId>& shots,
    const std::vector<map::TrackId>& tracks);
void AddConnections(map::TracksManager& manager, const map::ShotId& shot_id,
                    const std::vector<map::TrackId>& connections);
void RemoveConnections(map::TracksManager& manager, const map::ShotId& shot_id,
                       const std::vector<map::TrackId>& connections);
}  // namespace tracks_helpers
}  // namespace sfm
