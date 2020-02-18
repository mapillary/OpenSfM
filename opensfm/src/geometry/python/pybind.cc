#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include <geometry/essential.h>
#include <geometry/pose.h>
#include <geometry/triangulation.h>
#include <foundation/types.h>


PYBIND11_MODULE(pygeometry, m) {
  m.def("triangulate_bearings_dlt", geometry::TriangulateBearingsDLT);
  m.def("triangulate_bearings_midpoint", geometry::TriangulateBearingsMidpoint);
  m.def("triangulate_two_bearings_midpoint", geometry::TriangulateTwoBearingsMidpointSolve<double>);
  m.def("triangulate_two_bearings_midpoint_many", geometry::TriangulateTwoBearingsMidpointMany);
  m.def("essential_five_points", geometry::EssentialFivePoints);
  m.def("essential_n_points", geometry::EssentialNPoints);
  m.def("relative_pose_from_essential", geometry::RelativePoseFromEssential);
  m.def("relative_pose_refinement", geometry::RelativePoseRefinement);
}
