#include <geo/geo.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

PYBIND11_MODULE(pygeo, m) {
  m.def("ecef_from_lla", (Vec3d(*)(const Vec3d&))geo::EcefFromLla);
  m.def("ecef_from_lla",
        (Vec3d(*)(const double, const double, const double))geo::EcefFromLla);
  m.def("lla_from_ecef", (Vec3d(*)(const Vec3d&))geo::LlaFromEcef);
  m.def("lla_from_ecef",
        (Vec3d(*)(const double, const double, const double))geo::LlaFromEcef);

  m.def("ecef_from_topocentric_transform",
        (Mat4d(*)(const Vec3d&))geo::EcefFromTopocentricTransform);
  m.def("ecef_from_topocentric_transform",
        (Mat4d(*)(const double, const double,
                  const double))geo::EcefFromTopocentricTransform);

  m.def("ecef_from_topocentric_transform_finite_diff",
        (Mat4d(*)(const Vec3d&))geo::EcefFromTopocentricTransformFiniteDiff);
  m.def("ecef_from_topocentric_transform_finite_diff",
        (Mat4d(*)(const double, const double,
                  const double))geo::EcefFromTopocentricTransformFiniteDiff);

  m.def("topocentric_from_lla",
        (Vec3d(*)(const Vec3d&, const Vec3d&))geo::TopocentricFromLla);
  m.def("topocentric_from_lla",
        (Vec3d(*)(const double, const double, const double, const double,
                  const double, const double))geo::TopocentricFromLla);
  m.def("lla_from_topocentric",
        (Vec3d(*)(const Vec3d&, const Vec3d&))geo::LlaFromTopocentric);
  m.def("lla_from_topocentric",
        (Vec3d(*)(const double, const double, const double, const double,
                  const double, const double))geo::LlaFromTopocentric);
  m.def("gps_distance", geo::GpsDistance);

  py::class_<geo::TopocentricConverter>(m, "TopocentricConverter")
      .def(py::init<>())
      .def(py::init<const double, const double, const double>())
      .def_readonly("lat", &geo::TopocentricConverter::lat_)
      .def_readonly("lon", &geo::TopocentricConverter::long_)
      .def_readonly("alt", &geo::TopocentricConverter::alt_);
}
