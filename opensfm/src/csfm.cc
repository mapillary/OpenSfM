#include <boost/python.hpp>
#include <iostream>
#include <vector>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

#include "types.h"
#include "hahog.cc"
#include "multiview.cc"
#include "akaze.cc"
#include "bundle.h"
#include "openmvs_exporter.h"
#include "depthmap_wrapper.cc"

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

  def("triangulate_bearings_dlt", csfm::TriangulateBearingsDLT);
  def("triangulate_bearings_midpoint", csfm::TriangulateBearingsMidpoint);

  class_<BundleAdjuster, boost::noncopyable>("BundleAdjuster")
    .def("run", &BundleAdjuster::Run)
    .def("get_perspective_camera", &BundleAdjuster::GetPerspectiveCamera)
    .def("get_fisheye_camera", &BundleAdjuster::GetFisheyeCamera)
    .def("get_equirectangular_camera", &BundleAdjuster::GetEquirectangularCamera)
    .def("get_shot", &BundleAdjuster::GetShot)
    .def("get_point", &BundleAdjuster::GetPoint)
    .def("add_perspective_camera", &BundleAdjuster::AddPerspectiveCamera)
    .def("add_fisheye_camera", &BundleAdjuster::AddFisheyeCamera)
    .def("add_equirectangular_camera", &BundleAdjuster::AddEquirectangularCamera)
    .def("add_shot", &BundleAdjuster::AddShot)
    .def("add_point", &BundleAdjuster::AddPoint)
    .def("add_observation", &BundleAdjuster::AddObservation)
    .def("add_rotation_prior", &BundleAdjuster::AddRotationPrior)
    .def("add_translation_prior", &BundleAdjuster::AddTranslationPrior)
    .def("add_position_prior", &BundleAdjuster::AddPositionPrior)
    .def("add_point_position_prior", &BundleAdjuster::AddPointPositionPrior)
    .def("add_ground_control_point_observation", &BundleAdjuster::AddGroundControlPointObservation)
    .def("set_origin_shot", &BundleAdjuster::SetOriginShot)
    .def("set_unit_translation_shot", &BundleAdjuster::SetUnitTranslationShot)
    .def("set_loss_function", &BundleAdjuster::SetLossFunction)
    .def("set_reprojection_error_sd", &BundleAdjuster::SetReprojectionErrorSD)
    .def("set_max_num_iterations", &BundleAdjuster::SetMaxNumIterations)
    .def("set_num_threads", &BundleAdjuster::SetNumThreads)
    .def("set_internal_parameters_prior_sd", &BundleAdjuster::SetInternalParametersPriorSD)
    .def("set_compute_covariances", &BundleAdjuster::SetComputeCovariances)
    .def("get_covariance_estimation_valid", &BundleAdjuster::GetCovarianceEstimationValid)
    .def("set_compute_reprojection_errors", &BundleAdjuster::SetComputeReprojectionErrors)
    .def("brief_report", &BundleAdjuster::BriefReport)
    .def("full_report", &BundleAdjuster::FullReport)
  ;

  class_<BAPerspectiveCamera>("BAPerspectiveCamera")
    .add_property("focal", &BAPerspectiveCamera::GetFocal, &BAPerspectiveCamera::SetFocal)
    .add_property("k1", &BAPerspectiveCamera::GetK1, &BAPerspectiveCamera::SetK1)
    .add_property("k2", &BAPerspectiveCamera::GetK2, &BAPerspectiveCamera::SetK2)
    .def_readwrite("constant", &BAPerspectiveCamera::constant)
    .def_readwrite("focal_prior", &BAPerspectiveCamera::focal_prior)
    .def_readwrite("id", &BAPerspectiveCamera::id)
  ;

  class_<BAFisheyeCamera>("BAFisheyeCamera")
    .add_property("focal", &BAFisheyeCamera::GetFocal, &BAFisheyeCamera::SetFocal)
    .add_property("k1", &BAFisheyeCamera::GetK1, &BAFisheyeCamera::SetK1)
    .add_property("k2", &BAFisheyeCamera::GetK2, &BAFisheyeCamera::SetK2)
    .def_readwrite("constant", &BAFisheyeCamera::constant)
    .def_readwrite("focal_prior", &BAFisheyeCamera::focal_prior)
    .def_readwrite("id", &BAFisheyeCamera::id)
  ;

  class_<BAShot>("BAShot")
    .add_property("rx", &BAShot::GetRX, &BAShot::SetRX)
    .add_property("ry", &BAShot::GetRY, &BAShot::SetRY)
    .add_property("rz", &BAShot::GetRZ, &BAShot::SetRZ)
    .add_property("tx", &BAShot::GetTX, &BAShot::SetTX)
    .add_property("ty", &BAShot::GetTY, &BAShot::SetTY)
    .add_property("tz", &BAShot::GetTZ, &BAShot::SetTZ)
    .def("get_covariance", &BAShot::GetCovariance)
    .def_readwrite("constant", &BAShot::constant)
    .def_readwrite("camera", &BAShot::camera)
    .def_readwrite("id", &BAShot::id)
  ;

  class_<BAPoint>("BAPoint")
    .add_property("x", &BAPoint::GetX, &BAPoint::SetX)
    .add_property("y", &BAPoint::GetY, &BAPoint::SetY)
    .add_property("z", &BAPoint::GetZ, &BAPoint::SetZ)
    .def_readwrite("constant", &BAPoint::constant)
    .def_readwrite("reprojection_error", &BAPoint::reprojection_error)
    .def_readwrite("id", &BAPoint::id)
  ;

  class_<csfm::OpenMVSExporter>("OpenMVSExporter")
    .def("add_camera", &csfm::OpenMVSExporter::AddCamera)
    .def("add_shot", &csfm::OpenMVSExporter::AddShot)
    .def("add_point", &csfm::OpenMVSExporter::AddPoint)
    .def("export", &csfm::OpenMVSExporter::Export)
  ;

  class_<csfm::DepthmapEstimatorWrapper>("DepthmapEstimator")
    .def("set_depth_range", &csfm::DepthmapEstimatorWrapper::SetDepthRange)
    .def("set_patchmatch_iterations", &csfm::DepthmapEstimatorWrapper::SetPatchMatchIterations)
    .def("set_min_patch_sd", &csfm::DepthmapEstimatorWrapper::SetMinPatchSD)
    .def("add_view", &csfm::DepthmapEstimatorWrapper::AddView)
    .def("compute_patch_match", &csfm::DepthmapEstimatorWrapper::ComputePatchMatch)
    .def("compute_patch_match_sample", &csfm::DepthmapEstimatorWrapper::ComputePatchMatchSample)
    .def("compute_brute_force", &csfm::DepthmapEstimatorWrapper::ComputeBruteForce)
  ;

  class_<csfm::DepthmapCleanerWrapper>("DepthmapCleaner")
    .def("set_same_depth_threshold", &csfm::DepthmapCleanerWrapper::SetSameDepthThreshold)
    .def("set_min_consistent_views", &csfm::DepthmapCleanerWrapper::SetMinConsistentViews)
    .def("add_view", &csfm::DepthmapCleanerWrapper::AddView)
    .def("clean", &csfm::DepthmapCleanerWrapper::Clean)
  ;

  class_<csfm::DepthmapMergerWrapper>("DepthmapMerger")
    .def("set_same_depth_threshold", &csfm::DepthmapMergerWrapper::SetSameDepthThreshold)
    .def("add_view", &csfm::DepthmapMergerWrapper::AddView)
    .def("merge", &csfm::DepthmapMergerWrapper::Merge)
  ;
}
