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
         has_altitude: true if z coordinate is known
         observations: list of observations of the point on images
     */
  GroundControlPoint() = default;
  LandmarkId id_ = "";
  bool has_altitude_ = false;
  AlignedVector<GroundControlPointObservation> observations_;
  std::map<std::string, double> lla_;

  Vec3d GetLlaVec3d() const {
    return {
      lla_.at("latitude"),
      lla_.at("longitude"),
      has_altitude_ ? lla_.at("altitude") : 0.0,
    };
  }

  void SetLla(double lat, double lon, double alt) {
    lla_["latitude"] = lat;
    lla_["longitude"] = lon;
    lla_["altitude"] = alt;
    has_altitude_ = true;
  }

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
