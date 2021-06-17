#pragma once
#include <map/ground_control_points.h>
#include <map/map.h>
#include <pybind11/pybind11.h>
#include <sfm/tracks_manager.h>

#include <limits>
#include <unordered_set>

#include "map/defines.h"

namespace map {
namespace retriangulation {
void RealignPoints(const map::Map& reference,
                   const sfm::TracksManager& tracks_manager,
                   map::Map& to_align);
}  // namespace retriangulation
}  // namespace map
