#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include "essential.h"
#include "pose.h"
#include "triangulation.h"

namespace py = pybind11;


PYBIND11_MODULE(csfm, m) {
  google::InitGoogleLogging("csfm");

  m.def("triangulate_bearings_dlt", csfm::TriangulateBearingsDLT);
  m.def("triangulate_bearings_midpoint", csfm::TriangulateBearingsMidpoint);
  m.def("essential_five_points", csfm::EssentialFivePoints);
  m.def("essential_n_points", csfm::EssentialNPoints);
  m.def("relative_pose_from_essential", csfm::RelativePoseFromEssential);
}
