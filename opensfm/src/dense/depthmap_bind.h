#pragma once

#include <dense/depthmap.h>
#include <foundation/python_types.h>

using namespace foundation;

namespace dense {

class DepthmapEstimatorWrapper {
 public:
  void AddView(pyarray_d K, pyarray_d R, pyarray_d t, pyarray_uint8 image,
               pyarray_uint8 mask) {
    de_.AddView(K.data(), R.data(), t.data(), image.data(), mask.data(),
                image.shape(1), image.shape(0));
  }

  void SetDepthRange(double min_depth, double max_depth, int num_depth_planes) {
    de_.SetDepthRange(min_depth, max_depth, num_depth_planes);
  }

  void SetPatchMatchIterations(int n) { de_.SetPatchMatchIterations(n); }

  void SetPatchSize(int size) { de_.SetPatchSize(size); }

  void SetMinPatchSD(float sd) { de_.SetMinPatchSD(sd); }

  py::object ComputePatchMatch() {
    DepthmapEstimatorResult result;
    {
      py::gil_scoped_release release;
      de_.ComputePatchMatch(&result);
    }
    return ComputeReturnValues(result);
  }

  py::object ComputePatchMatchSample() {
    DepthmapEstimatorResult result;
    {
      py::gil_scoped_release release;
      de_.ComputePatchMatchSample(&result);
    }
    return ComputeReturnValues(result);
  }

  py::object ComputeBruteForce() {
    DepthmapEstimatorResult result;
    {
      py::gil_scoped_release release;
      de_.ComputeBruteForce(&result);
    }
    return ComputeReturnValues(result);
  }

  py::object ComputeReturnValues(const DepthmapEstimatorResult &result) {
    py::list retn;
    retn.append(py_array_from_data(result.depth.ptr<float>(0),
                                   result.depth.rows, result.depth.cols));
    retn.append(py_array_from_data(result.plane.ptr<float>(0),
                                   result.plane.rows, result.plane.cols, 3));
    retn.append(py_array_from_data(result.score.ptr<float>(0),
                                   result.score.rows, result.score.cols));
    retn.append(py_array_from_data(result.nghbr.ptr<int>(0), result.nghbr.rows,
                                   result.nghbr.cols));
    return std::move(retn);
  }

 private:
  DepthmapEstimator de_;
};

class DepthmapCleanerWrapper {
 public:
  void SetSameDepthThreshold(float t) { dc_.SetSameDepthThreshold(t); }

  void SetMinConsistentViews(int n) { dc_.SetMinConsistentViews(n); }

  void AddView(pyarray_d K, pyarray_d R, pyarray_d t, pyarray_f depth) {
    dc_.AddView(K.data(), R.data(), t.data(), depth.data(), depth.shape(1),
                depth.shape(0));
  }

  py::object Clean() {
    cv::Mat depth;
    {
      py::gil_scoped_release release;
      dc_.Clean(&depth);
    }
    return py_array_from_data(depth.ptr<float>(0), depth.rows, depth.cols);
  }

 private:
  DepthmapCleaner dc_;
};

class DepthmapPrunerWrapper {
 public:
  void SetSameDepthThreshold(float t) { dp_.SetSameDepthThreshold(t); }

  void AddView(pyarray_d K, pyarray_d R, pyarray_d t, pyarray_f depth,
               pyarray_f plane, pyarray_uint8 color, pyarray_uint8 label) {
    dp_.AddView(K.data(), R.data(), t.data(), depth.data(), plane.data(),
                color.data(), label.data(), depth.shape(1), depth.shape(0));
  }

  py::object Prune() {
    std::vector<float> points;
    std::vector<float> normals;
    std::vector<unsigned char> colors;
    std::vector<unsigned char> labels;

    {
      py::gil_scoped_release release;
      dp_.Prune(&points, &normals, &colors, &labels);
    }

    py::list retn;
    int n = int(points.size()) / 3;
    retn.append(py_array_from_data(&points[0], n, 3));
    retn.append(py_array_from_data(&normals[0], n, 3));
    retn.append(py_array_from_data(&colors[0], n, 3));
    retn.append(py_array_from_data(&labels[0], n));
    return std::move(retn);
  }

 private:
  DepthmapPruner dp_;
};

}  // namespace dense
