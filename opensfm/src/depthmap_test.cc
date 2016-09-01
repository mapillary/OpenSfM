#include <gtest/gtest.h>
#include "depthmap.cc"

namespace {

TEST(PlaneInducedHomography, ParallelCameras) {
  cv::Matx33d K1(600, 0, 0, 0, 400, 0, 0, 0, 1);
  cv::Matx33d R1(1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Matx31d t1(0, 0, 0);

  cv::Matx33d K2(400, 0, 0, 0, 300, 0, 0, 0, 1);
  cv::Matx33d R2(1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Matx31d t2(-1, 0, 0);

  cv::Matx31d v(0, 0, -1);      // plane z = 1
  cv::Matx31d p(0.1, 0.2, 1);   // a point in the plane

  // Compute point projections in image 1 and 2.
  cv::Matx31d p1 = K1 * (R1 * p + t1);
  float x1 = p1(0) / p1(2), y1 = p1(1) / p1(2);
  cv::Matx31d p2 = K2 * (R2 * p + t2);
  float x2 = p2(0) / p2(2), y2 = p2(1) / p2(2);

  // Warp point via plane homography.
  cv::Matx33d H = csfm::PlaneInducedHomography(K1, R1, t1, K2, R2, t2, v);
  float x2_mapped, y2_mapped;
  csfm::ApplyHomography(H, x1, y1, &x2_mapped, &y2_mapped);

  EXPECT_NEAR(x2, x2_mapped, 10e-8);
  EXPECT_NEAR(y2, y2_mapped, 10e-8);
}

}  // namespace
