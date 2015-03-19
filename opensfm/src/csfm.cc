#include <boost/python.hpp>
#include <iostream>
#include <vector>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h> 

#include "types.h"
#include "hahog.cc"
#include "multiview.cc"
#include "akaze.cc"
#include "dsift.cc"
#include "bundle.h"

#if (PY_VERSION_HEX < 0x03000000)
static void numpy_import_array_wrapper()
#else
static int* numpy_import_array_wrapper()
#endif
{
  /* Initialise numpy API and use 2/3 compatible return */
  import_array();
}

BOOST_PYTHON_MODULE(csfm) {
  using namespace boost::python;
  
  google::InitGoogleLogging("csfm");
  boost::python::numeric::array::set_module_and_type("numpy", "ndarray");
  numpy_import_array_wrapper();


  enum_<DESCRIPTOR_TYPE>("AkazeDescriptorType")
    .value("SURF_UPRIGHT", SURF_UPRIGHT)
    .value("SURF", SURF)
    .value("MSURF_UPRIGHT", MSURF_UPRIGHT)
    .value("MSURF", MSURF)
    .value("MLDB_UPRIGHT", MLDB_UPRIGHT)
    .value("MLDB", MLDB)
  ;

  class_<AKAZEOptions>("AKAZEOptions")
    .def_readwrite("omin", &AKAZEOptions::omin)
    .def_readwrite("omax", &AKAZEOptions::omax)
    .def_readwrite("nsublevels", &AKAZEOptions::nsublevels)
    .def_readwrite("img_width", &AKAZEOptions::img_width)
    .def_readwrite("img_height", &AKAZEOptions::img_height)
    .def_readwrite("soffset", &AKAZEOptions::soffset)
    .def_readwrite("derivative_factor", &AKAZEOptions::derivative_factor)
    .def_readwrite("sderivatives", &AKAZEOptions::sderivatives)
    .def_readwrite("diffusivity", &AKAZEOptions::diffusivity)
    .def_readwrite("dthreshold", &AKAZEOptions::dthreshold)
    .def_readwrite("min_dthreshold", &AKAZEOptions::min_dthreshold)
    .def_readwrite("target_num_features", &AKAZEOptions::target_num_features)
    .def_readwrite("use_adaptive_suppression", &AKAZEOptions::use_adaptive_suppression)
    .def_readwrite("descriptor", &AKAZEOptions::descriptor)
    .def_readwrite("descriptor_size", &AKAZEOptions::descriptor_size)
    .def_readwrite("descriptor_channels", &AKAZEOptions::descriptor_channels)
    .def_readwrite("descriptor_pattern_size", &AKAZEOptions::descriptor_pattern_size)
    .def_readwrite("kcontrast", &AKAZEOptions::kcontrast)
    .def_readwrite("kcontrast_percentile", &AKAZEOptions::kcontrast_percentile)
    .def_readwrite("kcontrast_nbins", &AKAZEOptions::kcontrast_nbins)
    .def_readwrite("use_isotropic_diffusion", &AKAZEOptions::use_isotropic_diffusion)
    .def_readwrite("save_scale_space", &AKAZEOptions::save_scale_space)
    .def_readwrite("save_keypoints", &AKAZEOptions::save_keypoints)
    .def_readwrite("verbosity", &AKAZEOptions::verbosity)
  ;

  def("akaze", csfm::akaze);

  def("hahog", csfm::hahog,
      (boost::python::arg("peak_threshold") = 0.003,
       boost::python::arg("edge_threshold") = 10,
       boost::python::arg("target_num_features") = 0,
       boost::python::arg("use_adaptive_suppression") = false
      )
  );

  def("dsift", csfm::dsift,
      (boost::python::arg("step") = 5,
       boost::python::arg("bin_size") = 5
      )
  );

  def("two_view_reconstruction", csfm::TwoViewReconstruction);

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
    .def("set_reprojection_error_sd", &BundleAdjuster::SetReprojectionErrorSD)
    .def("set_focal_prior_sd", &BundleAdjuster::SetFocalPriorSD)
    .def("brief_report", &BundleAdjuster::BriefReport)
    .def("full_report", &BundleAdjuster::FullReport)
  ;

  class_<BACamera>("BACamera")
    .add_property("focal", &BACamera::GetFocal, &BACamera::SetFocal)
    .add_property("k1", &BACamera::GetK1, &BACamera::SetK1)
    .add_property("k2", &BACamera::GetK2, &BACamera::SetK2)
    .def_readwrite("constant", &BACamera::constant)
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
    .def_readwrite("constant", &BAShot::constant)
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
    .def_readwrite("constant", &BAPoint::constant)
    .def_readwrite("id", &BAPoint::id)
  ;
}
