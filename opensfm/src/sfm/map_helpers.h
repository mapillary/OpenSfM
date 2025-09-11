#pragma once

#include <foundation/types.h>
#include <map/map.h>

namespace sfm::map_helpers {

int FilterBadlyConditionedPoints(map::Map& map, double min_angle_deg = 1.0,
                                 double min_abs_det = 1e-15);
int RemoveIsolatedPoints(map::Map& map, int k = 7);

}  // namespace sfm::map_helpers
