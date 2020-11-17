#pragma once
namespace map {

struct TopocentricConverter {
  TopocentricConverter() : lat_(0.0), long_(0.0), alt_(0.0) {}

  TopocentricConverter(const double lat, const double longitude,
                       const double alt)
      : lat_(lat), long_(longitude), alt_(alt) {}
  double lat_;
  double long_;
  double alt_;
};

}  // namespace map
