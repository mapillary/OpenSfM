#include <gtest/gtest.h>
#include "depthmap.cc"

namespace {

TEST(PlaneInducedHomography, ParallelCameras) {
  cv::Matx33d K1(600, 0, 300, 0, 400, 200, 0, 0, 1);
  cv::Matx33d R1;
  cv::Rodrigues(cv::Matx31d(0.1, 0.1, 0.1), R1);
  cv::Matx31d t1(0.1, 0.2, 0.3);

  cv::Matx33d K2(400, 0, 200, 0, 300, 150, 0, 0, 1);
  cv::Matx33d R2;
  cv::Rodrigues(cv::Matx31d(0.1, 0.2, 0.3), R2);
  cv::Matx31d t2(-0.3, -0.1, 0.2);

  cv::Matx31d v(0, 0, -1);                            // plane z = 1
  cv::Matx31d p_in_camera_coordinates(0.1, 0.2, 1);   // a point in the plane
  cv::Matx31d p = R1.t() * (p_in_camera_coordinates - t1);

  // Compute point projections in image 1 and 2.
  cv::Matx31d p1 = K1 * (R1 * p + t1);
  double x1 = p1(0) / p1(2), y1 = p1(1) / p1(2);
  cv::Matx31d p2 = K2 * (R2 * p + t2);
  double x2 = p2(0) / p2(2), y2 = p2(1) / p2(2);

  // Warp point via plane homography.
  cv::Matx33d H = csfm::PlaneInducedHomography(K1, R1, t1, K2, R2, t2, v);
  double x2_mapped, y2_mapped;
  csfm::ApplyHomography(H, x1, y1, &x2_mapped, &y2_mapped);

  EXPECT_NEAR(x2, x2_mapped, 10e-8);
  EXPECT_NEAR(y2, y2_mapped, 10e-8);
}

}  // namespace
