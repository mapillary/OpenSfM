#pragma once
#include <bundle/bundle_adjuster.h>
#include <map/map.h>
// TODO: Config

class BAHelpers {
public:
  static void SetUpBAFromReconstruction(map::Map& map, BundleAdjuster& ba);
  // next step
  // static void Bundle(map::Map& map, CONFIG);
};