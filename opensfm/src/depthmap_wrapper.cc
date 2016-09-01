
#include "types.h"
#include "depthmap.cc"


namespace csfm {

class DepthmapEstimatorWrapper {
 public:
  void AddView(PyObject *K,
               PyObject *R,
               PyObject *t,
               PyObject *image) {
    PyArrayContiguousView<double> K_view((PyArrayObject *)K);
    PyArrayContiguousView<double> R_view((PyArrayObject *)R);
    PyArrayContiguousView<double> t_view((PyArrayObject *)t);
    PyArrayContiguousView<unsigned char> image_view((PyArrayObject *)image);
    de_.AddView(K_view.data(), R_view.data(), t_view.data(),
                image_view.data(), image_view.shape(1), image_view.shape(0));
  }

  void SetDepthRange(double min_depth, double max_depth, int num_depth_planes) {
    de_.SetDepthRange(min_depth, max_depth, num_depth_planes);
  }

  bp::object Compute() {
    cv::Mat best_depth, best_score;
    de_.Compute(&best_depth, &best_score);

    bp::list retn;
    npy_intp shape[2] = {best_depth.rows, best_depth.cols};
    retn.append(bpn_array_from_data(2, shape, best_depth.ptr<float>(0)));
    retn.append(bpn_array_from_data(2, shape, best_score.ptr<float>(0)));
    return retn;
  }

 private:
  DepthmapEstimator de_;
};

}

