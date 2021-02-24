#pragma once
#include <foundation/optional.h>
#include <map/defines.h>
#include <map/map.h>
namespace map {
struct GroundControlPointObservation {
  /*    A ground control point observation.

       Attributes:
           shot_id: the shot where the point is observed
           projection: 2d coordinates of the observation
  **/
  GroundControlPointObservation() = default;
  GroundControlPointObservation(const ShotId& shot_id, const Vec2d& proj)
      : shot_id_(shot_id), projection_(proj) {}
  ShotId shot_id_ = "";
  Vec2d projection_ = Vec2d::Zero();
};
struct GroundControlPoint {
  /**A ground control point with its observations.

     Attributes:
         lla: latitue, longitude and altitude
         coordinates: x, y, z coordinates in topocentric reference frame
         has_altitude: true if z coordinate is known
         observations: list of observations of the point on images
     */
  GroundControlPoint() = default;
  LandmarkId id_ = "";
  foundation::OptionalValue<Vec3d> coordinates_;
  bool has_altitude_ = false;
  AlignedVector<GroundControlPointObservation> observations_;
  std::map<std::string, double> lla_;

  void SetObservations(
      const AlignedVector<GroundControlPointObservation>& obs) {
    observations_.clear();
    std::copy(obs.cbegin(), obs.cend(), std::back_inserter(observations_));
  }

  AlignedVector<GroundControlPointObservation> GetObservations() {
    return observations_;
  }

  void AddObservation(const GroundControlPointObservation& obs) {
    observations_.push_back(obs);
  }
};

}  // namespace map
