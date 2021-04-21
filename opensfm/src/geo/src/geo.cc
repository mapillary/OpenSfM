#include <geo/geo.h>

namespace geo {
double ToRadians(const double degrees) {
  constexpr auto deg_to_rad{3.14159265358979323846 / 180.0};
  return degrees * deg_to_rad;
}
double ToDegrees(const double radians) {
  constexpr auto rad_to_deg{180.0 / 3.14159265358979323846};
  return radians * rad_to_deg;
}

/**
    Compute ECEF XYZ from latitude, longitude and altitude.

    All using the WGS84 model.
    Altitude is the distance to the WGS84 ellipsoid.

*/
Vec3d EcefFromLla(const double lat, const double lon, const double alt) {
  constexpr auto a2 = WGS84_a * WGS84_a;
  constexpr auto b2 = WGS84_b * WGS84_b;
  const auto lat_rad = ToRadians(lat);
  const auto lon_rad = ToRadians(lon);
  const auto cos_lat = std::cos(lat_rad);
  const auto sin_lat = std::sin(lat_rad);
  const auto cos_lon = std::cos(lon_rad);
  const auto sin_lon = std::sin(lon_rad);
  const auto L =
      1.0 / std::sqrt(a2 * cos_lat * cos_lat + b2 * sin_lat * sin_lat);
  return Vec3d((a2 * L + alt) * cos_lat * cos_lon,
               (a2 * L + alt) * cos_lat * sin_lon, (b2 * L + alt) * sin_lat);
}

Vec3d EcefFromLla(const Vec3d& lla) {
  return EcefFromLla(lla[0], lla[1], lla[2]);
}

/**
Compute latitude, longitude and altitude from ECEF XYZ.

All using the WGS84 model.
Altitude is the distance to the WGS84 ellipsoid.
*/
Vec3d LlaFromEcef(const double x, const double y, const double z) {
  constexpr auto a2 = WGS84_a * WGS84_a;
  constexpr auto b2 = WGS84_b * WGS84_b;
  const auto ea = std::sqrt((a2 - b2) / a2);
  const auto eb = std::sqrt((a2 - b2) / b2);
  const auto p = std::sqrt(x * x + y * y);
  const auto theta = std::atan2(z * WGS84_a, p * WGS84_b);
  const auto lon = std::atan2(y, x);
  const auto lat =
      std::atan2(z + eb * eb * WGS84_b * std::pow(std::sin(theta), 3),
                 p - ea * ea * WGS84_a * std::pow(std::cos(theta), 3));
  const auto N =
      WGS84_a / std::sqrt(1.0 - ea * ea * std::pow(std::sin(lat), 2));
  const auto alt = p / std::cos(lat) - N;
  return Vec3d(ToDegrees(lat), ToDegrees(lon), alt);
}
Vec3d LlaFromEcef(const Vec3d& ecef) {
  return LlaFromEcef(ecef[0], ecef[1], ecef[2]);
}

/**
    Transformation from a topocentric frame at reference position to ECEF.

    The topocentric reference frame is a metric one with the origin
    at the given (lat, lon, alt) position, with the X axis heading east,
    the Y axis heading north and the Z axis vertical to the ellipsoid.
*/
Mat4d EcefFromTopocentricTransform(const double lat, const double lon,
                                   const double alt) {
  const Vec3d ecef = EcefFromLla(lat, lon, alt);

  const auto sa = std::sin(ToRadians(lat));
  const auto ca = std::cos(ToRadians(lat));
  const auto so = std::sin(ToRadians(lon));
  const auto co = std::cos(ToRadians(lon));
  Mat4d ecefT;
  ecefT << -so, -sa * co, ca * co, ecef[0], co, -sa * so, ca * so, ecef[1], 0,
      ca, sa, ecef[2], 0, 0, 0, 1;
  return ecefT;
}

Mat4d EcefFromTopocentricTransform(const Vec3d& lla) {
  return EcefFromTopocentricTransform(lla[0], lla[1], lla[2]);
}

/**
    Transformation from a topocentric frame at reference position to ECEF.

    The topocentric reference frame is a metric one with the origin
    at the given (lat, lon, alt) position, with the X axis heading east,
    the Y axis heading north and the Z axis vertical to the ellipsoid.
*/
Mat4d EcefFromTopocentricTransformFiniteDiff(const double lat, const double lon,
                                             const double alt) {
  constexpr auto eps{1e-2};
  const Vec3d ecef = EcefFromLla(lat, lon, alt);
  const Vec3d v1 =
      ((EcefFromLla(lat, lon + eps, alt) - EcefFromLla(lat, lon - eps, alt)) /
       (2 * eps))
          .normalized();
  const Vec3d v2 =
      ((EcefFromLla(lat + eps, lon, alt) - EcefFromLla(lat - eps, lon, alt)) /
       (2 * eps))
          .normalized();
  const Vec3d v3 =
      ((EcefFromLla(lat, lon, alt + eps) - EcefFromLla(lat, lon, alt - eps)) /
       (2 * eps))
          .normalized();
  Mat4d ecefT;
  ecefT << v1[0], v2[0], v3[0], ecef[0], v1[1], v2[1], v3[1], ecef[1], v1[2],
      v2[2], v3[2], ecef[2], 0, 0, 0, 1;
  return ecefT;
}

Mat4d EcefFromTopocentricTransformFiniteDiff(const Vec3d& lla) {
  return EcefFromTopocentricTransformFiniteDiff(lla[0], lla[1], lla[2]);
}

Vec3d TopocentricFromLla(const double lat, const double lon, const double alt,
                         const double reflat, const double reflon,
                         const double refalt) {
  return TopocentricFromLla(Vec3d(lat, lon, alt),
                            Vec3d(reflat, reflon, refalt));
}

Vec3d TopocentricFromLla(const Vec3d& lla, const Vec3d& ref) {
  const Mat4d T = EcefFromTopocentricTransform(ref).inverse();
  const Vec3d xyz = EcefFromLla(lla);
  return T.block<3, 4>(0, 0) * xyz.homogeneous();
}

/**
    Transform from topocentric XYZ to lat, lon, alt.
*/
Vec3d LlaFromTopocentric(const Vec3d& xyz, const Vec3d& ref) {
  const Mat4d T = EcefFromTopocentricTransform(ref);
  return LlaFromEcef(T.block<3, 4>(0, 0) * xyz.homogeneous());
}

Vec3d LlaFromTopocentric(const double x, const double y, const double z,
                         const double reflat, const double reflon,
                         const double refalt) {
  return LlaFromTopocentric(Vec3d(x, y, z), Vec3d(reflat, reflon, refalt));
}

/**
    Distance between two (lat,lon) pairs.
*/
double GpsDistance(const Vec2d& lat_lon1, const Vec2d& lat_lon2) {
  const Vec3d dist = EcefFromLla(lat_lon1[0], lat_lon1[1], 0.0) -
                     EcefFromLla(lat_lon2[0], lat_lon2[1], 0.0);
  return dist.norm();
}

TopocentricConverter::TopocentricConverter()
    : lat_(0.0), long_(0.0), alt_(0.0) {}

TopocentricConverter::TopocentricConverter(const double lat,
                                           const double longitude,
                                           const double alt)
    : lat_(lat), long_(longitude), alt_(alt) {}

TopocentricConverter::TopocentricConverter(const Vec3d& lla)
    : TopocentricConverter(lla[0], lla[1], lla[2]){};

Vec3d TopocentricConverter::ToTopocentric(const double lat, const double lon,
                                          const double alt) const {
  return ToTopocentric(Vec3d(lat, lon, alt));
}

Vec3d TopocentricConverter::ToTopocentric(const Vec3d& lla) const {
  return TopocentricFromLla(lla, GetLlaRef());
}

Vec3d TopocentricConverter::ToLla(const double x, const double y,
                                  const double z) const {
  return ToLla(Vec3d(x, y, z));
}

Vec3d TopocentricConverter::ToLla(const Vec3d& xyz) const {
  return LlaFromTopocentric(xyz, GetLlaRef());
}

Vec3d TopocentricConverter::GetLlaRef() const {
  return Vec3d(lat_, long_, alt_);
}
}  // namespace geo
