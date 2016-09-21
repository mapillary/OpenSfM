
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

  bp::object ComputePatchMatch() {
    cv::Mat depth, plane, score;
    de_.ComputePatchMatch(&depth, &plane, &score);
    return ComputeReturnValues(depth, plane, score);
  }

  bp::object ComputeBruteForce() {
    cv::Mat depth, plane, score;
    de_.ComputeBruteForce(&depth, &plane, &score);
    return ComputeReturnValues(depth, plane, score);
  }

  bp::object ComputeReturnValues(const cv::Mat &depth,
                                 const cv::Mat &plane,
                                 const cv::Mat &score) {
    bp::list retn;
    npy_intp shape[2] = {depth.rows, depth.cols};
    npy_intp plane_shape[3] = {depth.rows, depth.cols, 3};
    retn.append(bpn_array_from_data(2, shape, depth.ptr<float>(0)));
    retn.append(bpn_array_from_data(3, plane_shape, plane.ptr<float>(0)));
    retn.append(bpn_array_from_data(2, shape, score.ptr<float>(0)));
    return retn;
  }

 private:
  DepthmapEstimator de_;
};


class DepthmapCleanerWrapper {
 public:
  void AddView(PyObject *K,
               PyObject *R,
               PyObject *t,
               PyObject *depth) {
    PyArrayContiguousView<double> K_view((PyArrayObject *)K);
    PyArrayContiguousView<double> R_view((PyArrayObject *)R);
    PyArrayContiguousView<double> t_view((PyArrayObject *)t);
    PyArrayContiguousView<float> depth_view((PyArrayObject *)depth);
    dc_.AddView(K_view.data(), R_view.data(), t_view.data(),
                depth_view.data(), depth_view.shape(1), depth_view.shape(0));
  }

  bp::object Clean() {
    cv::Mat depth;
    dc_.Clean(&depth);
    npy_intp shape[2] = {depth.rows, depth.cols};
    return bpn_array_from_data(2, shape, depth.ptr<float>(0));
  }

 private:
  DepthmapCleaner dc_;
};

class DepthmapMergerWrapper {
 public:
  void AddView(PyObject *K,
               PyObject *R,
               PyObject *t,
               PyObject *depth,
               PyObject *normal,
               PyObject *color) {
    PyArrayContiguousView<double> K_view((PyArrayObject *)K);
    PyArrayContiguousView<double> R_view((PyArrayObject *)R);
    PyArrayContiguousView<double> t_view((PyArrayObject *)t);
    PyArrayContiguousView<float> depth_view((PyArrayObject *)depth);
    PyArrayContiguousView<float> plane_view((PyArrayObject *)normal);
    PyArrayContiguousView<unsigned char> color_view((PyArrayObject *)color);
    dm_.AddView(K_view.data(), R_view.data(), t_view.data(),
                depth_view.data(), plane_view.data(), color_view.data(),
                depth_view.shape(1), depth_view.shape(0));
  }

  bp::object Merge() {
    std::vector<float> points;
    std::vector<float> normals;
    std::vector<unsigned char> colors;

    dm_.Merge(&points, &normals, &colors);

    bp::list retn;
    npy_intp shape[2] = {int(points.size()) / 3, 3};
    retn.append(bpn_array_from_data(2, shape, &points[0]));
    retn.append(bpn_array_from_data(2, shape, &normals[0]));
    retn.append(bpn_array_from_data(2, shape, &colors[0]));
    return retn;
  }

 private:
  DepthmapMerger dm_;
};

}

