#include <random>

#include <opencv2/opencv.hpp>

namespace csfm {

bool IsInsideImage(const cv::Mat &image, int i, int j) {
  return i >= 0 && i < image.rows && j >= 0 && j < image.cols;
}

template<typename T>
float LinearInterpolation(const cv::Mat &image, float y, float x) {
  int ix = int(x);
  int iy = int(y);
  if (ix < 0 || ix + 1 >= image.cols || iy < 0 || iy + 1 >= image.rows) {
    return 0.0f;
  }
  float dx = x - ix;
  float dy = y - iy;
  float im00 = image.at<T>(iy, ix);
  float im01 = image.at<T>(iy, ix + 1);
  float im10 = image.at<T>(iy + 1, ix);
  float im11 = image.at<T>(iy + 1, ix + 1);
  float im0 = (1 - dx) * im00 + dx * im01;
  float im1 = (1 - dx) * im10 + dx * im11;
  return (1 - dy) * im0 + dy * im1;
}

float Variance(float *x, int n) {
  float sum = 0;
  for (int i = 0; i < n; ++i) {
    sum += x[i];
  }
  float mean = sum / n;

  float sum2 = 0;
  for (int i = 0; i < n; ++i) {
    sum2 += (x[i] - mean) * (x[i] - mean);
  }
  return sum2 / n;
}

class NCCEstimator {
 public:
  NCCEstimator()
    : sumx_(0), sumy_(0)
    , sumxx_(0), sumyy_(0), sumxy_(0)
    , sumw_(0)
  {}

  void Push(float x, float y) {
    sumx_ += x;
    sumy_ += y;
    sumxx_ += x * x;
    sumyy_ += y * y;
    sumxy_ += x * y;
    sumw_ += 1;
  }

  float Get() {
    float meanx = sumx_ / sumw_;
    float meany = sumy_ / sumw_;
    float meanxx = sumxx_ / sumw_;
    float meanyy = sumyy_ / sumw_;
    float meanxy = sumxy_ / sumw_;
    float varx = meanxx - meanx * meanx;
    float vary = meanyy - meany * meany;
    return (meanxy - meanx * meany) / sqrt(varx * vary + 1e-10);
  }

 private:
  float sumx_, sumy_;
  float sumxx_, sumyy_, sumxy_;
  float sumw_;
};

void ApplyHomography(const cv::Matx33f &H,
                     float x1, float y1,
                     float *x2, float *y2) {
  float w = H(2, 0) * x1 + H(2, 1) * y1 + H(2, 2);
  *x2 = (H(0, 0) * x1 + H(0, 1) * y1 + H(0, 2)) / w;
  *y2 = (H(1, 0) * x1 + H(1, 1) * y1 + H(1, 2)) / w;
}

cv::Matx33d PlaneInducedHomography(const cv::Matx33d K1,
                                   const cv::Matx33d R1,
                                   const cv::Vec3d t1,
                                   const cv::Matx33d K2,
                                   const cv::Matx33d R2,
                                   const cv::Vec3d t2,
                                   const cv::Vec3d v) {
  cv::Matx33d R2R1 = R2 * R1.t();
  return K2 * (R2R1 + (R2R1 * t1 - t2) * v.t()) * K1.inv();
}

cv::Matx33f PlaneInducedHomographyBaked(const cv::Matx33d K1inv,
                                        const cv::Matx33d Q2,
                                        const cv::Vec3d a2,
                                        const cv::Matx33d K2,
                                        const cv::Vec3d v) {
  return K2 * (Q2 + a2 * v.t()) * K1inv;
}

cv::Vec3d Project(cv::Vec3d x,
                  const cv::Matx33d &K,
                  const cv::Matx33d &R,
                  const cv::Vec3d &t) {
  return K * (R * x + t);
}

cv::Vec3d Backproject(double x, double y, double depth,
                      const cv::Matx33d &K,
                      const cv::Matx33d &R,
                      const cv::Vec3d &t) {
  return R.t() * (depth * K.inv() * cv::Vec3d(x, y, 1) - t);
}

float DepthOfPlaneBackprojection(double x, double y,
                                 const cv::Matx33d &K,
                                 const cv::Vec3d &plane) {
  float denom  = -(plane.t() * K.inv() * cv::Vec3d(x, y, 1))(0);
  return 1.0f / std::max(1e-6f, denom);
}

cv::Vec3f PlaneFromDepthAndNormal(float x, float y,
                                  const cv::Matx33d &K,
                                  float depth,
                                  const cv::Vec3f &normal) {
  cv::Vec3f point = depth * K.inv() * cv::Vec3d(x, y, 1);
  float denom = -normal.dot(point);
  return normal / std::max(1e-6f, denom);
}

float UniformRand(float a, float b) {
  return a + (b - a) * float(rand()) / RAND_MAX;
}

class DepthmapEstimator {
 public:
  DepthmapEstimator()
    : patch_size_(7)
    , min_depth_(0)
    , max_depth_(0)
    , num_depth_planes_(50)
    , patchmatch_iterations_(3)
    , min_patch_variance_(5 * 5)
    , rng_{std::random_device{}()}
    , uni_(0, 0)
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
    Kinvs_.emplace_back(Ks_.back().inv());
    Qs_.emplace_back(Rs_.back() * Rs_.front().t());
    as_.emplace_back(Qs_.back() * ts_.front() - ts_.back());
    images_.emplace_back(cv::Mat(height, width, CV_8U, (void *)pimage).clone());
    std::size_t size = images_.size();
    int a = (size > 1) ? 1 : 0;
    int b = (size > 1) ? size - 1 : 0;
    uni_.param(std::uniform_int_distribution<int>::param_type(a, b));
  }

  void SetDepthRange(double min_depth, double max_depth, int num_depth_planes) {
    min_depth_ = min_depth;
    max_depth_ = max_depth;
    num_depth_planes_ = num_depth_planes;
  }

  void SetPatchMatchIterations(int n) {
    patchmatch_iterations_ = n;
  }

  void SetMinPatchSD(float sd) {
    min_patch_variance_ = sd * sd;
  }

  void ComputeBruteForce(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr) {
    AssignMatrices(best_depth, best_plane, best_score, best_nghbr);

    int hpz = (patch_size_ - 1) / 2;
    for (int i = hpz; i < best_depth->rows - hpz; ++i) {
      for (int j = hpz; j < best_depth->cols - hpz; ++j) {
        for (int d = 0; d < num_depth_planes_; ++d) {
          float depth = 1 / (1 / min_depth_ + d * (1 / max_depth_ - 1 / min_depth_) / (num_depth_planes_ - 1));
          cv::Vec3f normal(0, 0, -1);
          cv::Vec3f plane = PlaneFromDepthAndNormal(j, i, Ks_[0], depth, normal);
          CheckPlaneCandidate(best_depth, best_plane, best_score, best_nghbr, i, j, plane);
        }
      }
    }
  }

  void ComputePatchMatch(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr) {
    AssignMatrices(best_depth, best_plane, best_score, best_nghbr);
    RandomInitialization(best_depth, best_plane, best_score, best_nghbr, false);
    ComputeIgnoreMask(best_depth, best_plane, best_score, best_nghbr);

    for (int i = 0; i < patchmatch_iterations_; ++i) {
      PatchMatchForwardPass(best_depth, best_plane, best_score, best_nghbr, false);
      PatchMatchBackwardPass(best_depth, best_plane, best_score, best_nghbr, false);
    }
  }

  void ComputePatchMatchSample(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr) {
    AssignMatrices(best_depth, best_plane, best_score, best_nghbr);
    RandomInitialization(best_depth, best_plane, best_score, best_nghbr, true);
    ComputeIgnoreMask(best_depth, best_plane, best_score, best_nghbr);

    for (int i = 0; i < patchmatch_iterations_; ++i) {
      PatchMatchForwardPass(best_depth, best_plane, best_score, best_nghbr, true);
      PatchMatchBackwardPass(best_depth, best_plane, best_score, best_nghbr, true);
    }
  }

  void AssignMatrices(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr) {
    *best_depth = cv::Mat(images_[0].rows, images_[0].cols, CV_32F, 0.0f);
    *best_plane = cv::Mat(images_[0].rows, images_[0].cols, CV_32FC3, 0.0f);
    *best_score = cv::Mat(images_[0].rows, images_[0].cols, CV_32F, 0.0f);
    *best_nghbr = cv::Mat(images_[0].rows, images_[0].cols, CV_32S, cv::Scalar(0));
  }

  void RandomInitialization(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr, bool sample) {
    int hpz = (patch_size_ - 1) / 2;
    for (int i = hpz; i < best_depth->rows - hpz; ++i) {
      for (int j = hpz; j < best_depth->cols - hpz; ++j) {
        float depth = UniformRand(min_depth_, max_depth_);
        cv::Vec3f normal(UniformRand(-1, 1), UniformRand(-1, 1), -1);
        cv::Vec3f plane = PlaneFromDepthAndNormal(j, i, Ks_[0], depth, normal);
        int nghbr;
        float score;
        if (sample) {
          nghbr = uni_(rng_);
          score = ComputePlaneImageScore(i, j, plane, nghbr);
        } else {
          ComputePlaneScore(i, j, plane, &score, &nghbr);
        }
        AssignPixel(best_depth, best_plane, best_score, best_nghbr, i, j, depth, plane, score, nghbr);
      }
    }
  }

  void ComputeIgnoreMask(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr) {
    int hpz = (patch_size_ - 1) / 2;
    for (int i = hpz; i < best_depth->rows - hpz; ++i) {
      for (int j = hpz; j < best_depth->cols - hpz; ++j) {
        float variance = PatchVariance(i, j);
        if (variance < min_patch_variance_) {
          AssignPixel(best_depth, best_plane, best_score, best_nghbr, i, j, 0.0f, cv::Vec3f(0, 0, 0), 0.0f, 0);
        }
      }
    }
  }

  float PatchVariance(int i, int j) {
    float patch[patch_size_ * patch_size_];
    int hpz = (patch_size_ - 1) / 2;
    int counter = 0;
    for (int u = -hpz; u <= hpz; ++u) {
      for (int v = -hpz; v <= hpz; ++v) {
        patch[counter++] = images_[0].at<unsigned char>(i + u, j + v);
      }
    }
    return Variance(patch, patch_size_ * patch_size_);
  }


  void PatchMatchForwardPass(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr,
                             bool sample) {
    int adjacent[2][2] = {{-1, 0}, {0, -1}};
    int hpz = (patch_size_ - 1) / 2;
    for (int i = hpz; i < best_depth->rows - hpz; ++i) {
      for (int j = hpz; j < best_depth->cols - hpz; ++j) {
        PatchMatchUpdatePixel(best_depth, best_plane, best_score, best_nghbr, i, j, adjacent, sample);
      }
    }
  }

  void PatchMatchBackwardPass(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr,
                              bool sample) {
    int adjacent[2][2] = {{0, 1}, {1, 0}};
    int hpz = (patch_size_ - 1) / 2;
    for (int i = best_depth->rows - hpz - 1; i >= hpz; --i) {
      for (int j = best_depth->cols - hpz - 1; j >= hpz; --j) {
        PatchMatchUpdatePixel(best_depth, best_plane, best_score, best_nghbr, i, j, adjacent, sample);
      }
    }
  }

  void PatchMatchUpdatePixel(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr,
                             int i, int j,
                             int adjacent[2][2],
                             bool sample) {
    // Ignore pixels with depth == 0.
    if (best_depth->at<float>(i, j) == 0.0f) {
      return;
    }

    // Check neighbors and their planes for adjacent pixels.
    for (int k = 0; k < 2; ++k) {
      int i_adjacent = i + adjacent[k][0];
      int j_adjacent = j + adjacent[k][1];

      // Do not propagate ignored adjacent pixels.
      if (best_depth->at<float>(i_adjacent, j_adjacent) == 0.0f) {
        continue;
      }

      cv::Vec3f plane = best_plane->at<cv::Vec3f>(i_adjacent, j_adjacent);

      if (sample) {
        int nghbr = best_nghbr->at<int>(i_adjacent, j_adjacent);
        CheckPlaneImageCandidate(best_depth, best_plane, best_score, best_nghbr, i, j, plane, nghbr);
      } else {
        CheckPlaneCandidate(best_depth, best_plane, best_score, best_nghbr, i, j, plane);
      }
    }

    // Check random planes for current neighbor.
    float depth_range = (1 / max_depth_ - 1 / min_depth_) / 20;
    float normal_range = 0.5;
    int current_nghbr = best_nghbr->at<int>(i, j);
    for (int k = 0; k < 6; ++k) {
      float current_depth = best_depth->at<float>(i, j);
      float depth = 1 / (1 / current_depth + UniformRand(-depth_range, depth_range));

      cv::Vec3f current_plane = best_plane->at<cv::Vec3f>(i, j);
      cv::Vec3f normal(-current_plane(0) / current_plane(2) + UniformRand(-normal_range, normal_range),
                       -current_plane(1) / current_plane(2) + UniformRand(-normal_range, normal_range),
                       -1.0f);

      cv::Vec3f plane = PlaneFromDepthAndNormal(j, i, Ks_[0], depth, normal);
      if (sample) {
        CheckPlaneImageCandidate(best_depth, best_plane, best_score, best_nghbr, i, j, plane, current_nghbr);
      } else {
        CheckPlaneCandidate(best_depth, best_plane, best_score, best_nghbr, i, j, plane);
      }

      depth_range *= 0.5;
      normal_range *= 0.5;
    }

    if (!sample || images_.size() <= 2) {
      return;
    }

    // Check random other neighbor for current plane.
    int other_nghbr = uni_(rng_);
    while (other_nghbr == current_nghbr) {
      other_nghbr = uni_(rng_);
    }

    cv::Vec3f plane = best_plane->at<cv::Vec3f>(i, j);
    CheckPlaneImageCandidate(best_depth, best_plane, best_score, best_nghbr, i, j, plane, other_nghbr);
  }

  void CheckPlaneCandidate(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr,
                           int i, int j, const cv::Vec3f &plane) {
    float score;
    int nghbr;
    ComputePlaneScore(i, j, plane, &score, &nghbr);
    if (score > best_score->at<float>(i, j)) {
      float depth = DepthOfPlaneBackprojection(j, i, Ks_[0], plane);
      AssignPixel(best_depth, best_plane, best_score, best_nghbr, i, j, depth, plane, score, nghbr);
    }
  }

  void CheckPlaneImageCandidate(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr,
                                int i, int j, const cv::Vec3f &plane, int nghbr) {
    float score = ComputePlaneImageScore(i, j, plane, nghbr);
    if (score > best_score->at<float>(i, j)) {
      float depth = DepthOfPlaneBackprojection(j, i, Ks_[0], plane);
      AssignPixel(best_depth, best_plane, best_score, best_nghbr, i, j, depth, plane, score, nghbr);
    }
  }

  void AssignPixel(cv::Mat *best_depth, cv::Mat *best_plane, cv::Mat *best_score, cv::Mat *best_nghbr,
                   int i, int j, const float depth, const cv::Vec3f &plane, const float score, const int nghbr) {
      best_depth->at<float>(i, j) = depth;
      best_plane->at<cv::Vec3f>(i, j) = plane;
      best_score->at<float>(i, j) = score;
      best_nghbr->at<int>(i, j) = nghbr;
  }

  void ComputePlaneScore(int i, int j, const cv::Vec3f &plane, float *score, int *nghbr) {
    *score = -1.0f;
    *nghbr = 0;
    for (int other = 1; other < images_.size(); ++other) {
      float image_score = ComputePlaneImageScore(i, j, plane, other);
      if (image_score > *score) {
        *score = image_score;
        *nghbr = other;
      }
    }
  }

  float ComputePlaneImageScoreUnoptimized(int i, int j,
                                          const cv::Vec3f &plane,
                                          int other) {
    cv::Matx33f H = PlaneInducedHomographyBaked(
        Kinvs_[0], Qs_[other], as_[other], Ks_[other], plane);
    int hpz = (patch_size_ - 1) / 2;
    NCCEstimator ncc;
    for (int u = -hpz; u <= hpz; ++u) {
      for (int v = -hpz; v <= hpz; ++v) {
        float im1 = images_[0].at<unsigned char>(i + u, j + v);
        float x2, y2;
        ApplyHomography(H, j + v, i + u, &x2, &y2);
        float im2 = LinearInterpolation<unsigned char>(images_[other], y2, x2);
        ncc.Push(im1, im2);
      }
    }
    return ncc.Get();
  }

  float ComputePlaneImageScore(int i, int j,
                               const cv::Vec3f &plane,
                               int other) {
    cv::Matx33f H = PlaneInducedHomographyBaked(
        Kinvs_[0], Qs_[other], as_[other], Ks_[other], plane);
    int hpz = (patch_size_ - 1) / 2;

    float u = H(0, 0) * j + H(0, 1) * i + H(0, 2);
    float v = H(1, 0) * j + H(1, 1) * i + H(1, 2);
    float w = H(2, 0) * j + H(2, 1) * i + H(2, 2);

    float dfdx_x = (H(0, 0) * w - H(2, 0) * u) / (w * w);
    float dfdx_y = (H(1, 0) * w - H(2, 0) * v) / (w * w);
    float dfdy_x = (H(0, 1) * w - H(2, 1) * u) / (w * w);
    float dfdy_y = (H(1, 1) * w - H(2, 1) * v) / (w * w);

    float Hx0 = u / w;
    float Hy0 = v / w;

    NCCEstimator ncc;
    for (int dy = -hpz; dy <= hpz; ++dy) {
      for (int dx = -hpz; dx <= hpz; ++dx) {
        float im1 = images_[0].at<unsigned char>(i + dy, j + dx);
        float x2 = Hx0 + dfdx_x * dx + dfdy_x * dy;
        float y2 = Hy0 + dfdx_y * dx + dfdy_y * dy;
        float im2 = LinearInterpolation<unsigned char>(images_[other], y2, x2);
        ncc.Push(im1, im2);
      }
    }
    return ncc.Get();
  }


 private:
  std::vector<cv::Mat> images_;
  std::vector<cv::Matx33d> Ks_;
  std::vector<cv::Matx33d> Rs_;
  std::vector<cv::Vec3d> ts_;
  std::vector<cv::Matx33d> Kinvs_;
  std::vector<cv::Matx33d> Qs_;
  std::vector<cv::Vec3d> as_;
  int patch_size_;
  double min_depth_, max_depth_;
  int num_depth_planes_;
  int patchmatch_iterations_;
  float min_patch_variance_;
  std::mt19937 rng_;
  std::uniform_int_distribution<int> uni_;
};


class DepthmapCleaner {
 public:
  DepthmapCleaner()
    : same_depth_threshold_(0.01)
    , min_consistent_views_(2)
  {}

  void SetSameDepthThreshold(float t) {
    same_depth_threshold_ = t;
  }

  void SetMinConsistentViews(int n) {
    min_consistent_views_ = n;
  }

  void AddView(const double *pK,
               const double *pR,
               const double *pt,
               const float *pdepth,
               int width,
               int height) {
    Ks_.emplace_back(pK);
    Rs_.emplace_back(pR);
    ts_.emplace_back(pt);
    depths_.emplace_back(cv::Mat(height, width, CV_32F, (void *)pdepth).clone());
  }

  void Clean(cv::Mat *clean_depth) {
    *clean_depth = cv::Mat(depths_[0].rows, depths_[0].cols, CV_32F, 0.0f);

    for (int i = 0; i < depths_[0].rows; ++i) {
      for (int j = 0; j < depths_[0].cols; ++j) {
        float depth = depths_[0].at<float>(i, j);
        cv::Vec3f point = Backproject(j, i, depth, Ks_[0], Rs_[0], ts_[0]);
        int consistent_views = 1;
        for (int other = 1; other < depths_.size(); ++other) {
          cv::Vec3f reprojection = Project(point, Ks_[other], Rs_[other], ts_[other]);
          float u = reprojection(0) / reprojection(2);
          float v = reprojection(1) / reprojection(2);
          float depth_of_point = reprojection(2);
          float depth_at_reprojection = LinearInterpolation<float>(depths_[other], v, u);
          if (fabs(depth_at_reprojection - depth_of_point) < depth_of_point * same_depth_threshold_) {
            consistent_views++;
          }
        }
        if (consistent_views >= min_consistent_views_) {
          clean_depth->at<float>(i, j) = depths_[0].at<float>(i, j);
        } else {
          clean_depth->at<float>(i, j) = 0;
        }
      }
    }
  }

 private:
  std::vector<cv::Mat> depths_;
  std::vector<cv::Matx33d> Ks_;
  std::vector<cv::Matx33d> Rs_;
  std::vector<cv::Vec3d> ts_;
  float same_depth_threshold_;
  int min_consistent_views_;
};


class DepthmapPruner {
 public:
  DepthmapPruner()
    : same_depth_threshold_(0.01)
  {}

  void SetSameDepthThreshold(float t) {
    same_depth_threshold_ = t;
  }

  void AddView(const double *pK,
               const double *pR,
               const double *pt,
               const float *pdepth,
               const float *pplane,
               const unsigned char *pcolor,
               int width,
               int height) {
     Ks_.emplace_back(pK);
     Rs_.emplace_back(pR);
     ts_.emplace_back(pt);
     depths_.emplace_back(cv::Mat(height, width, CV_32F, (void *)pdepth).clone());
     planes_.emplace_back(cv::Mat(height, width, CV_32FC3, (void *)pplane).clone());
     colors_.emplace_back(cv::Mat(height, width, CV_8UC3, (void *)pcolor).clone());
  }

  void Prune(std::vector<float> *merged_points,
             std::vector<float> *merged_normals,
             std::vector<unsigned char> *merged_colors) {
    cv::Matx33f Rinv = Rs_[0].t();
    for (int i = 0; i < depths_[0].rows; ++i) {
      for (int j = 0; j < depths_[0].cols; ++j) {
        float depth = depths_[0].at<float>(i, j);
        if (depth <= 0) {
          continue;
        }
        cv::Vec3f normal = cv::normalize(planes_[0].at<cv::Vec3f>(i, j));
        float area = -normal(2) / depth * Ks_[0](0, 0);
        cv::Vec3f point = Backproject(j, i, depth, Ks_[0], Rs_[0], ts_[0]);
        bool keep = true;
        for (int other = 1; other < depths_.size(); ++other) {
          cv::Vec3d reprojection = Project(point, Ks_[other], Rs_[other], ts_[other]);
          int iu = int(reprojection(0) / reprojection(2) + 0.5f);
          int iv = int(reprojection(1) / reprojection(2) + 0.5f);
          float depth_of_point = reprojection(2);
          if (!IsInsideImage(depths_[other], iv, iu)) {
            continue;
          }
          float depth_at_reprojection = depths_[other].at<float>(iv, iu);
          if (depth_at_reprojection > (1 - same_depth_threshold_) * depth_of_point) {
            cv::Vec3f normal_at_reprojection = cv::normalize(planes_[other].at<cv::Vec3f>(iv, iu));
            float area_at_reprojection = -normal_at_reprojection(2) / depth_at_reprojection * Ks_[other](0, 0);
            if (area_at_reprojection > area) {
              keep = false;
              break;
            }
          }
        }
        if (keep) {
          cv::Vec3f R1_normal = Rinv * normal;
          cv::Vec3b color = colors_[0].at<cv::Vec3b>(i, j);
          merged_points->push_back(point[0]);
          merged_points->push_back(point[1]);
          merged_points->push_back(point[2]);
          merged_normals->push_back(R1_normal[0]);
          merged_normals->push_back(R1_normal[1]);
          merged_normals->push_back(R1_normal[2]);
          merged_colors->push_back(color[0]);
          merged_colors->push_back(color[1]);
          merged_colors->push_back(color[2]);
        }
      }
    }
  }

 private:
  std::vector<cv::Mat> depths_;
  std::vector<cv::Mat> planes_;
  std::vector<cv::Mat> colors_;
  std::vector<cv::Matx33d> Ks_;
  std::vector<cv::Matx33d> Rs_;
  std::vector<cv::Vec3d> ts_;
  float same_depth_threshold_;
};

}

