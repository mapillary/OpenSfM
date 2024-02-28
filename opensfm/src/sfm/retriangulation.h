#pragma once
#include <map/map.h>
#include <map/tracks_manager.h>

namespace sfm::retriangulation {
void RealignMaps(const map::Map& reference, map::Map& to_align,
                 bool update_points);
}  // namespace sfm
