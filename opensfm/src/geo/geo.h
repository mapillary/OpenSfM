#pragma once
#include <foundation/types.h>

namespace geo {
constexpr double WGS84_a = 6378137.0;
constexpr double WGS84_b = 6356752.314245;
Vec3d EcefFromLla(const Vec3d& lla);
Vec3d EcefFromLla(const double lat, const double lon, const double alt);
Vec3d LlaFromEcef(const Vec3d& ecef);
Vec3d LlaFromEcef(const double x, const double y, const double z);

Mat4d EcefFromTopocentricTransform(const Vec3d& lla);
Mat4d EcefFromTopocentricTransform(const double lat, const double lon,
                                   const double alt);
Mat4d EcefFromTopocentricTransformFiniteDiff(const Vec3d& lla);
Mat4d EcefFromTopocentricTransformFiniteDiff(const double lat, const double lon,
                                             const double alt);
Vec3d TopocentricFromLla(const double lat, const double lon, const double alt,
                         const double reflat, const double reflon,
                         const double refalt);

Vec3d TopocentricFromLla(const Vec3d& lla, const Vec3d& ref);

Vec3d LlaFromTopocentric(const Vec3d& xyz, const Vec3d& ref);

Vec3d LlaFromTopocentric(const double x, const double y, const double z,
                         const double reflat, const double reflon,
                         const double refalt);
double GpsDistance(const Vec2d& lat_lon1, const Vec2d& lat_lon2);

double ToRadians(const double degrees);
double ToDegrees(const double radians);

struct TopocentricConverter {
  TopocentricConverter();

  TopocentricConverter(const double lat, const double longitude,
                       const double alt);
  explicit TopocentricConverter(const Vec3d& lla);
  double lat_;
  double long_;
  double alt_;
  Vec3d ToTopocentric(const double lat, const double lon,
                      const double alt) const;
  Vec3d ToTopocentric(const Vec3d& lla) const;
  Vec3d ToLla(const double x, const double y, const double z) const;
  Vec3d ToLla(const Vec3d& xyz) const;
  Vec3d GetLlaRef() const;
};
};  // namespace geo
