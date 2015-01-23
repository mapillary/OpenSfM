#include <Python.h> // This must be included before anything else. See http://bugs.python.org/issue10910
#include <boost/python.hpp>
#include <iostream>
#include <vector>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h> 

#include "types.h"
#include "hahog.cc"
#include "bundle.h"


BOOST_PYTHON_MODULE(csfm) {
  using namespace boost::python;
  
  google::InitGoogleLogging("csfm");
  boost::python::numeric::array::set_module_and_type("numpy", "ndarray");
  import_array();

  def("hahog", csfm::hahog,
      (arg("peak_threshold") = 0.003,
       arg("edge_threshold") = 10
      )
  );

  class_<BundleAdjuster>("BundleAdjuster")
    .def("run", &BundleAdjuster::Run)
    .def("get_camera", &BundleAdjuster::GetCamera)
    .def("get_shot", &BundleAdjuster::GetShot)
    .def("get_point", &BundleAdjuster::GetPoint)
    .def("add_camera", &BundleAdjuster::AddCamera)
    .def("add_shot", &BundleAdjuster::AddShot)
    .def("add_point", &BundleAdjuster::AddPoint)
    .def("add_observation", &BundleAdjuster::AddObservation)
    .def("set_loss_function", &BundleAdjuster::SetLossFunction)
    .def("set_focal_prior_sd", &BundleAdjuster::SetFocalPriorSD)
  ;

  class_<BACamera>("BACamera")
    .add_property("focal", &BACamera::GetFocal, &BACamera::SetFocal)
    .add_property("k1", &BACamera::GetK1, &BACamera::SetK1)
    .add_property("k2", &BACamera::GetK2, &BACamera::SetK2)
    .def_readwrite("width", &BACamera::width)
    .def_readwrite("height", &BACamera::height)
    .def_readwrite("exif_focal", &BACamera::exif_focal)
    .def_readwrite("id", &BACamera::id)
  ;
  
  class_<BAShot>("BAShot")
    .add_property("rx", &BAShot::GetRX, &BAShot::SetRX)
    .add_property("ry", &BAShot::GetRY, &BAShot::SetRY)
    .add_property("rz", &BAShot::GetRZ, &BAShot::SetRZ)
    .add_property("tx", &BAShot::GetTX, &BAShot::SetTX)
    .add_property("ty", &BAShot::GetTY, &BAShot::SetTY)
    .add_property("tz", &BAShot::GetTZ, &BAShot::SetTZ)
    .def_readwrite("gps_x", &BAShot::gps_x)
    .def_readwrite("gps_y", &BAShot::gps_y)
    .def_readwrite("gps_z", &BAShot::gps_z)
    .def_readwrite("gps_dop", &BAShot::gps_dop)
    .def_readwrite("camera", &BAShot::camera)
    .def_readwrite("id", &BAShot::id)
  ;

  class_<BAPoint>("BAPoint")
    .add_property("x", &BAPoint::GetX, &BAPoint::SetX)
    .add_property("y", &BAPoint::GetY, &BAPoint::SetY)
    .add_property("z", &BAPoint::GetZ, &BAPoint::SetZ)
    .def_readwrite("id", &BAPoint::id)
  ;
}
