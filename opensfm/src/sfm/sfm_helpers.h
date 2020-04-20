#pragma once

#include <sfm/observation.h>
#include <sfm/tracks_manager.h>
#include <sfm/types.h>


namespace sfm_helpers {

std::unordered_map<ShotId, int> CountTracksPerShot(
    const TracksManager& manager, const std::vector<ShotId>& shots,
    const std::vector<TrackId>& tracks);
}  // namespace sfm_helpers