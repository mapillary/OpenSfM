
#include "types.h"


namespace csfm {

class DepthmapEstimator {
 public:
  void AddView(const double *pK,
               const double *pR,
               const double *pt,
               const unsigned char *pimage,
               int width,
               int height) {
    cv::Matx33d K(pK);
    cv::Matx33d R(pR);
    cv::Matx31d t(pt);
    Ks_.emplace_back(K);
    Rs_.emplace_back(R);
    ts_.emplace_back(t);
    cv::Matx34d Rt;
    cv::hconcat(R, t, Rt);
    Ps_.emplace_back(K * Rt);

    images_.emplace_back(cv::Mat(height, width, CV_8U, (void *)pimage));
  }

  void Compute() {
    cv::Mat best_depth(images_[0].rows, images_[0].cols, CV_32F, 0.0f);
    cv::Mat best_score(images_[0].rows, images_[0].cols, CV_32F, -1.0f);

    int num_depth_tests = 10;
    float min_depth = 1;
    float max_depth = 10;

    for (int i = 0; i < best_depth.rows; ++i) {
      for (int j = 0; j < best_depth.cols; ++j) {
        for (int d = 0; d < num_depth_tests; ++d) {
          float depth = min_depth + d * (max_depth - min_depth) / (num_depth_tests - 1);
          float score = ComputePlaneScore(i, j, depth);
          if (score > best_score.at<float>(i, j)) {
            best_score.at<float>(i, j) = score;
            best_depth.at<float>(i, j) = depth;
          }
        }
        return;
      }
    }
  }

  float ComputePlaneScore(int i, int j, float depth) {
    float best_score = -1.0f;
    for (int other = 1; other < images_.size(); ++other) {
      float score = ComputePlaneImageScore(i, j, depth, other);
      if (score > best_score) {
        best_score = score;
      }
    }
    return best_score;
  }

  float ComputePlaneImageScore(int i, int j, float depth, int other) {
    cv::Matx33d H = PlaneInducedHomography(Ks_[0], Rs_[0], ts_[0],
                                           Ks_[other], Rs_[other], ts_[other],
                                           cv::Matx31d(0, 0, -1 / depth));
    return 0;
  }

  cv::Matx33d PlaneInducedHomography(const cv::Matx33d K1,
                                     const cv::Matx33d R1,
                                     const cv::Matx31d t1,
                                     const cv::Matx33d K2,
                                     const cv::Matx33d R2,
                                     const cv::Matx31d t2,
                                     const cv::Matx31d v) {
    cv::Matx33d R2R1 = R2 * R1.t();
    return K2 * (R2R1 + (R2R1 * t1 - t2) * v.t()) * K1.inv();
  }

 private:
  std::vector<cv::Mat> images_;
  std::vector<cv::Matx33d> Ks_;
  std::vector<cv::Matx33d> Rs_;
  std::vector<cv::Matx31d> ts_;
  std::vector<cv::Matx34d> Ps_;
};


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

  void Compute() {
    de_.Compute();
  }

 private:
  DepthmapEstimator de_;
};


}

