#pragma once
#include <map/map.h>
#include <map/tracks_manager.h>

namespace sfm {
namespace retriangulation {
void RealignPoints(const map::Map& reference,
                   map::Map& to_align);
}  // namespace retriangulation
}  // namespace sfm
