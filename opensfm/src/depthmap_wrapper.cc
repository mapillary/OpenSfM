
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

  void SetPatchMatchIterations(int n) {
    de_.SetPatchMatchIterations(n);
  }

  void SetMinPatchSD(float sd) {
    de_.SetMinPatchSD(sd);
  }

  bp::object ComputePatchMatch() {
    cv::Mat depth, plane, score, nghbr;
    de_.ComputePatchMatch(&depth, &plane, &score, &nghbr);
    return ComputeReturnValues(depth, plane, score, nghbr);
  }

  bp::object ComputePatchMatchSample() {
    cv::Mat depth, plane, score, nghbr;
    de_.ComputePatchMatchSample(&depth, &plane, &score, &nghbr);
    return ComputeReturnValues(depth, plane, score, nghbr);
  }

  bp::object ComputeBruteForce() {
    cv::Mat depth, plane, score, nghbr;
    de_.ComputeBruteForce(&depth, &plane, &score, &nghbr);
    return ComputeReturnValues(depth, plane, score, nghbr);
  }

  bp::object ComputeReturnValues(const cv::Mat &depth,
                                 const cv::Mat &plane,
                                 const cv::Mat &score,
                                 const cv::Mat &nghbr) {
    bp::list retn;
    retn.append(bpn_array_from_data(depth.ptr<float>(0), depth.rows, depth.cols));
    retn.append(bpn_array_from_data(plane.ptr<float>(0), plane.rows, plane.cols, 3));
    retn.append(bpn_array_from_data(score.ptr<float>(0), score.rows, score.cols));
    retn.append(bpn_array_from_data(nghbr.ptr<int>(0), nghbr.rows, nghbr.cols));
    return retn;
  }

 private:
  DepthmapEstimator de_;
};


class DepthmapCleanerWrapper {
 public:
  void SetSameDepthThreshold(float t) {
    dc_.SetSameDepthThreshold(t);
  }

  void SetMinConsistentViews(int n) {
    dc_.SetMinConsistentViews(n);
  }

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
    return bpn_array_from_data(depth.ptr<float>(0), depth.rows, depth.cols);
  }

 private:
  DepthmapCleaner dc_;
};


class DepthmapPrunerWrapper {
 public:
  void SetSameDepthThreshold(float t) {
    dp_.SetSameDepthThreshold(t);
  }

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
    dp_.AddView(K_view.data(), R_view.data(), t_view.data(),
                depth_view.data(), plane_view.data(), color_view.data(),
                depth_view.shape(1), depth_view.shape(0));
  }

  bp::object Prune() {
    std::vector<float> points;
    std::vector<float> normals;
    std::vector<unsigned char> colors;

    dp_.Prune(&points, &normals, &colors);

    bp::list retn;
    int n = int(points.size()) / 3;
    retn.append(bpn_array_from_data(&points[0], n, 3));
    retn.append(bpn_array_from_data(&normals[0], n, 3));
    retn.append(bpn_array_from_data(&colors[0], n, 3));
    return retn;
  }

 private:
  DepthmapPruner dp_;
};

}

