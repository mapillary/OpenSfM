
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
    Ks_.emplace_back(pK);
    Rs_.emplace_back(pR);
    ts_.emplace_back(pt);
    images_.emplace_back(height, width, CV_8U, (void *)pimage);
  }

  void Compute() {
    cv::Mat best_depth(images_[0].rows, images_[0].cols, CV_32F, 0.0f);
    cv::Mat best_score(images_[0].rows, images_[0].cols, CV_32F, -1.0f);

    int num_depth_tests = 10;
    float min_depth = 1;
    float max_depth = 10;

    for (int i = 0; i < best_depth.rows; ++i) {
      std::cout << "i " << i << "\n";
      for (int j = 0; j < best_depth.cols; ++j) {
        for (int d = 0; d < num_depth_tests; ++d) {
          float depth = min_depth + d * (max_depth - min_depth) / (num_depth_tests - 1);
          float score = ComputePlaneScore(i, j, depth);
          if (score > best_score.at<float>(i, j)) {
            best_score.at<float>(i, j) = score;
            best_depth.at<float>(i, j) = depth;
          }
        }
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

    int patch_size = 7;
    int hpz = (patch_size - 1) / 2;
    float patch1[patch_size * patch_size];
    float patch2[patch_size * patch_size];
    int counter = 0;
    for (int u = -hpz; u <= hpz; ++u) {
      for (int v = -hpz; v <= hpz; ++v) {
        patch1[counter] = images_[0].at<unsigned char>(i + u, j + v);
        float x2, y2;
        ApplyHomography(H, j + v, i + u, &x2, &y2);
        patch2[counter] = LinearInterpolation(images_[other], y2, x2);
        counter++;
      }
    }
    return NormalizedCrossCorrelation(patch1, patch2, patch_size * patch_size);
  }

  float LinearInterpolation(const cv::Mat &image, float y, float x) {
    int ix = int(x);
    int iy = int(y);
    float dx = x - ix;
    float dy = y - iy;
    float im00 = image.at<unsigned char>(iy, ix);
    float im01 = image.at<unsigned char>(iy, ix + 1);
    float im10 = image.at<unsigned char>(iy + 1, ix);
    float im11 = image.at<unsigned char>(iy + 1, ix + 1);
    float im0 = (1 - dx) * im00 + dx * im01;
    float im1 = (1 - dx) * im10 + dx * im11;
    return (1 - dy) * im0 + dy * im1;
  }

  float NormalizedCrossCorrelation(float *x, float *y, int n) {
    float sumx = 0;
    float sumx2 = 0;
    float sumy = 0;
    float sumy2 = 0;
    for (int i = 0; i < n; ++i) {
      sumx += x[i];
      sumx2 += x[i] * x[i];
      sumy += y[i];
      sumy2 += y[i] * y[i];
    }
    float meanx = sumx / n;
    float varx =  sumx2 / n - meanx * meanx; // E(x^2) - E(x)^2
    float meany = sumy / n;
    float vary =  sumy2 / n - meany * meany;
    float correlation = 0;
    for (int i = 0; i < n; ++i) {
      correlation += (x[i] - meanx) * (y[i] - meany) / sqrt(varx * vary);
    }
    return correlation;
  }

  void ApplyHomography(const cv::Matx33d &H,
                       float x1, float y1,
                       float *x2, float *y2) {
    float w = H(2, 0) * x1 + H(2, 1) * y1 + H(2, 2);
    *x2 = (H(0, 0) * x1 + H(0, 1) * y1 + H(0, 2)) / w;
    *y2 = (H(1, 0) * x1 + H(1, 1) * y1 + H(1, 2)) / w;
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

