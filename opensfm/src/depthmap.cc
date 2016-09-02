
#include  <opencv2/opencv.hpp>


namespace csfm {


float LinearInterpolation(const cv::Mat &image, float y, float x) {
  int ix = int(x);
  int iy = int(y);
  if (ix < 0 || ix + 1 >= image.cols || iy < 0 || iy + 1 >= image.rows) {
    return 0.0f;
  }
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
  float sumy = 0;
  for (int i = 0; i < n; ++i) {
    sumx += x[i];
    sumy += y[i];
  }
  float meanx = sumx / n;
  float meany = sumy / n;

  float sumx2 = 0;
  float sumy2 = 0;
  for (int i = 0; i < n; ++i) {
    sumx2 += (x[i] - meanx) * (x[i] - meanx);
    sumy2 += (y[i] - meany) * (y[i] - meany);
  }
  float varx =  sumx2 / n;
  float vary =  sumy2 / n;

  float correlation = 0;
  for (int i = 0; i < n; ++i) {
    correlation += (x[i] - meanx) * (y[i] - meany) / sqrt(varx * vary + 1e-10);
  }
  return correlation / n;
}

void ApplyHomography(const cv::Matx33d &H,
                     double x1, double y1,
                     double *x2, double *y2) {
  double w = H(2, 0) * x1 + H(2, 1) * y1 + H(2, 2);
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


class DepthmapEstimator {
 public:
  DepthmapEstimator()
   : patch_size_(7)
   , min_depth_(0)
   , max_depth_(0)
   , num_depth_planes_(50)
   {}

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

  void SetDepthRange(double min_depth, double max_depth, int num_depth_planes) {
    min_depth_ = min_depth;
    max_depth_ = max_depth;
    num_depth_planes_ = num_depth_planes;
  }

  void Compute(cv::Mat *best_depth, cv::Mat *best_score) {
    *best_depth = cv::Mat(images_[0].rows, images_[0].cols, CV_32F, 0.0f);
    *best_score = cv::Mat(images_[0].rows, images_[0].cols, CV_32F, -1.0f);

    int hpz = (patch_size_ - 1) / 2;
    for (int i = hpz; i < best_depth->rows - hpz; ++i) {
      std::cout << "i " << i << "\n";
      for (int j = hpz; j < best_depth->cols - hpz; ++j) {
        for (int d = 0; d < num_depth_planes_; ++d) {
          float depth = min_depth_ + d * (max_depth_ - min_depth_) / (num_depth_planes_ - 1);
          float score = ComputePlaneScore(i, j, depth);
          if (score > best_score->at<float>(i, j)) {
            best_score->at<float>(i, j) = score;
            best_depth->at<float>(i, j) = depth;
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
    int hpz = (patch_size_ - 1) / 2;
    float patch1[patch_size_ * patch_size_];
    float patch2[patch_size_ * patch_size_];
    int counter = 0;
    for (int u = -hpz; u <= hpz; ++u) {
      for (int v = -hpz; v <= hpz; ++v) {
        patch1[counter] = images_[0].at<unsigned char>(i + u, j + v);
        double x2, y2;
        ApplyHomography(H, j + v, i + u, &x2, &y2);
        patch2[counter] = LinearInterpolation(images_[other], y2, x2);
        counter++;
      }
    }
    return NormalizedCrossCorrelation(patch1, patch2, patch_size_ * patch_size_);
  }

 private:
  std::vector<cv::Mat> images_;
  std::vector<cv::Matx33d> Ks_;
  std::vector<cv::Matx33d> Rs_;
  std::vector<cv::Matx31d> ts_;
  int patch_size_;
  double min_depth_, max_depth_;
  int num_depth_planes_;
};


}

