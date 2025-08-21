#pragma once
#include <unordered_set>

#include <map/map.h>
#include <map/tracks_manager.h>

namespace sfm::retriangulation {
void RealignMaps(const map::Map& reference, map::Map& to_align,
                 bool update_points);
int Triangulate(map::Map& map, const std::unordered_set<map::TrackId>& track_ids, float min_angle, float min_depth, int processing_threads);
}  // namespace sfm::retriangulation
