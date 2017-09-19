
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
    npy_intp shape[2] = {depth.rows, depth.cols};
    npy_intp plane_shape[3] = {depth.rows, depth.cols, 3};
    retn.append(bpn_array_from_data(2, shape, depth.ptr<float>(0)));
    retn.append(bpn_array_from_data(3, plane_shape, plane.ptr<float>(0)));
    retn.append(bpn_array_from_data(2, shape, score.ptr<float>(0)));
    retn.append(bpn_array_from_data(2, shape, nghbr.ptr<int>(0)));
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
    npy_intp shape[2] = {depth.rows, depth.cols};
    return bpn_array_from_data(2, shape, depth.ptr<float>(0));
  }

 private:
  DepthmapCleaner dc_;
};

class DepthmapMergerWrapper {
 public:
  void SetSameDepthThreshold(float t) {
    dm_.SetSameDepthThreshold(t);
  }

  void AddView(PyObject *K,
               PyObject *R,
               PyObject *t,
               PyObject *depth,
               PyObject *normal,
               PyObject *color,
               PyObject *label,
               bp::object neighbors) {
    PyArrayContiguousView<double> K_view((PyArrayObject *)K);
    PyArrayContiguousView<double> R_view((PyArrayObject *)R);
    PyArrayContiguousView<double> t_view((PyArrayObject *)t);
    PyArrayContiguousView<float> depth_view((PyArrayObject *)depth);
    PyArrayContiguousView<float> plane_view((PyArrayObject *)normal);
    PyArrayContiguousView<unsigned char> color_view((PyArrayObject *)color);
    PyArrayContiguousView<unsigned char> label_view((PyArrayObject *)label);
    std::vector<int> neighbors_vector;
    for (int i = 0; i < bp::len(neighbors); ++i) {
      neighbors_vector.push_back(bp::extract<int>(neighbors[i]));
    }
    dm_.AddView(K_view.data(), R_view.data(), t_view.data(),
                depth_view.data(), plane_view.data(),
                color_view.data(), label_view.data(),
                neighbors_vector,
                depth_view.shape(1), depth_view.shape(0));
  }

  bp::object Merge() {
    std::vector<float> points;
    std::vector<float> normals;
    std::vector<unsigned char> colors;
    std::vector<unsigned char> labels;

    dm_.Merge(&points, &normals, &colors, &labels);

    bp::list retn;
    npy_intp shape3[2] = {int(points.size()) / 3, 3};
    npy_intp shape1[1] = {int(points.size()) / 3};
    retn.append(bpn_array_from_data(2, shape3, &points[0]));
    retn.append(bpn_array_from_data(2, shape3, &normals[0]));
    retn.append(bpn_array_from_data(2, shape3, &colors[0]));
    retn.append(bpn_array_from_data(1, shape1, &labels[0]));
    return retn;
  }

 private:
  DepthmapMerger dm_;
};

}

