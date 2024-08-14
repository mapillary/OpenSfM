#pragma once
#include <foundation/optional.h>
#include <map/defines.h>
#include <map/map.h>
namespace map {
enum GroundControlPointRole {
  /*    A ground control point role in SfM cluster merge and bundle adjuster

       Attributes:
           OPTIMIZATION: used in the optimization of map chunks and logging metrics
           METRICS_ONLY: only used in logging metrics

  **/
  OPTIMIZATION = 0,
  METRICS_ONLY = 1
};
struct GroundControlPointObservation {
  /*    A ground control point observation.

       Attributes:
           shot_id: the shot where the point is observed
           projection: 2d coordinates of the observation
           uid: a unique id for this observation
  **/
  GroundControlPointObservation() = default;
  GroundControlPointObservation(const ShotId& shot_id, const Vec2d& proj)
      : shot_id_(shot_id), projection_(proj) {}
  ShotId shot_id_;
  Vec2d projection_ = Vec2d::Zero();
  LandmarkUniqueId uid_ = 0;
};
struct GroundControlPoint {
  /**A ground control point with its observations.

     Attributes:
         lla: latitude, longitude and altitude
         has_altitude: true if z coordinate is known
         observations: list of observations of the point on images
         id: a unique id for this point group (survey point + image observations)
         survey_point_id: a unique id for the point on the ground
         role: OPTIMIZATION if gcp is used in SfM optimization, METRICS_ONLY if the
         ground control point is used for computing map merging metrics
     */
  GroundControlPoint() = default;
  LandmarkId id_;
  LandmarkUniqueId survey_point_id_ = 0;
  bool has_altitude_ = false;
  AlignedVector<GroundControlPointObservation> observations_;
  std::map<std::string, double> lla_;
  GroundControlPointRole role_;

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
