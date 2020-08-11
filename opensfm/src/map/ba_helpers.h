#pragma once
#include <pybind11/pybind11.h>
#include <bundle/bundle_adjuster.h>
#include <map/map.h>
#include <map/ground_control_points.h>

namespace py = pybind11;
namespace map
{
  class GroundControlPoint;
}
class OpenSfMConfig;
class BAHelpers {
public:
  static void SetUpBAFromReconstruction(map::Map& map, BundleAdjuster& ba);
  // next step
//   static py::dict Bundle(
//       map::Map& map,
//       const std::unordered_set<map::CameraId, Camera>& camera_priors,
//       const std::vector<map::GroundControlPoints>& gcp,
//       const OpenSfMConfig& config);
  
  static py::dict Bundle(map::Map& map, const OpenSfMConfig& config);
  static py::tuple BundleLocal(map::Map& map,
                               const map::ShotId& central_shot_id,
                               const OpenSfMConfig& config);
  static std::pair<std::unordered_set<map::Shot*>,
                   std::unordered_set<map::Shot*>>
  ShotNeighborhood(map::Map& map, const map::ShotId& central_shot_id,
                   const size_t radius, const size_t min_common_points,
                   const size_t max_interior_size);
private:
  //Implement const get_shot
  static std::unordered_set<map::Shot*> DirectShotNeighbors(
      map::Map& map, const std::unordered_set<map::Shot*>& shot_ids,
      const size_t min_common_points, const size_t max_neighbors);
  static AddGCPToBundle(BundleAdjuster& ba, const AlignedVector<map::GroundControlPoint>& gcp,
                        const std::unordered_map<ShotId, Shot>& shots);
  static bool TriangulateGCP(const map::GroundControlPoint& point,
      const std::unordered_map<ShotId, Shot>& shots, Vec3d& coordinates);
};