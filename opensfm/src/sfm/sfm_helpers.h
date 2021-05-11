#pragma once

#include <sfm/observation.h>
#include <sfm/tracks_manager.h>
#include <sfm/types.h>

namespace sfm_helpers {

std::unordered_map<ShotId, int> CountTracksPerShot(
    const TracksManager& manager, const std::vector<ShotId>& shots,
    const std::vector<TrackId>& tracks);
void AddConnections(TracksManager& manager, const ShotId& shot_id,
                    const std::vector<TrackId>& connections);
void RemoveConnections(TracksManager& manager, const ShotId& shot_id,
                       const std::vector<TrackId>& connections);
}  // namespace sfm_helpers
