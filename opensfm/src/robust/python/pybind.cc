#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include <foundation/types.h>
#include <robust/robust_estimator.h>
#include <robust/scorer.h>
#include <robust/instanciations.h>


template <class T>
void AddScoreType(py::module& m, const std::string& name) {
  py::class_<ScoreInfo<T>>(m, ("ScoreInfo" + name).c_str())
      .def(py::init())
      .def_readwrite("score", &ScoreInfo<T>::score)
      .def_readwrite("model", &ScoreInfo<T>::model)
      .def_readwrite("lo_model", &ScoreInfo<T>::lo_model)
      .def_readwrite("inliers_indices", &ScoreInfo<T>::inliers_indices)
    ;
  }

PYBIND11_MODULE(pyrobust, m) {
  py::class_<RobustEstimatorParams>(m, "RobustEstimatorParams")
    .def(py::init())
    .def_readwrite("iterations", &RobustEstimatorParams::iterations)
    .def_readwrite("probability", &RobustEstimatorParams::probability)
    .def_readwrite("use_local_optimization", &RobustEstimatorParams::use_local_optimization)
    .def_readwrite("use_iteration_reduction", &RobustEstimatorParams::use_iteration_reduction)
  ;

  m.def("ransac_line", robust::RANSACLine);
  m.def("ransac_essential", robust::RANSACEssential);
  m.def("ransac_relative_pose", robust::RANSACRelativePose);
  m.def("ransac_relative_rotation", robust::RANSACRelativeRotation);
  m.def("ransac_absolute_pose", robust::RANSACAbsolutePose);
  m.def("ransac_absolute_pose_known_rotation", robust::RANSACAbsolutePoseKnownRotation);


  py::enum_<RansacType>(m, "RansacType")
      .value("RANSAC", RansacType::RANSAC)
      .value("MSAC", RansacType::MSAC)
      .value("LMedS", RansacType::LMedS)
      .export_values()
    ;
  AddScoreType<Line::Type>(m, "Line");
  AddScoreType<Eigen::Matrix3d>(m, "Matrix3d");
  AddScoreType<Eigen::Matrix<double, 3, 4>>(m, "Matrix34d");
  AddScoreType<Eigen::Vector3d>(m, "Vector3d");
}
