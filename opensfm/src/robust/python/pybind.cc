#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include "robust_estimator.h"
#include "scorer.h"
#include "instanciations.h"


namespace py = pybind11;


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

PYBIND11_MODULE(csfm, m) {
  google::InitGoogleLogging("csfm");

  py::class_<RobustEstimatorParams>(m, "RobustEstimatorParams")
    .def(py::init())
    .def_readwrite("iterations", &RobustEstimatorParams::iterations)
    .def_readwrite("probability", &RobustEstimatorParams::probability)
    .def_readwrite("use_local_optimization", &RobustEstimatorParams::use_local_optimization)
    .def_readwrite("use_iteration_reduction", &RobustEstimatorParams::use_iteration_reduction)
  ;

  m.def("ransac_line", csfm::RANSACLine);
  m.def("ransac_essential", csfm::RANSACEssential);
  m.def("ransac_relative_pose", csfm::RANSACRelativePose);

  py::enum_<RansacType>(m, "RansacType")
      .value("RANSAC", RansacType::RANSAC)
      .value("MSAC", RansacType::MSAC)
      .value("LMedS", RansacType::LMedS)
      .export_values()
    ;
  AddScoreType<Line::MODEL>(m, "Line");
  AddScoreType<csfm::EssentialMatrixModel::MODEL>(m, "EssentialMatrix");
  AddScoreType<Eigen::Matrix<double, 3, 4>>(m, "RelativePose");
}
