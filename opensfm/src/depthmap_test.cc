#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <gtest/gtest.h>
#include "depthmap.cc"

namespace {

using namespace csfm;


TEST(PlaneInducedHomography, RandomPoint) {
  cv::Matx33d K1(600, 0, 300, 0, 400, 200, 0, 0, 1);
  cv::Matx33d R1;
  cv::Rodrigues(cv::Vec3d(0.1, 0.1, 0.1), R1);
  cv::Vec3d t1(0.1, 0.2, 0.3);

  cv::Matx33d K2(400, 0, 200, 0, 300, 150, 0, 0, 1);
  cv::Matx33d R2;
  cv::Rodrigues(cv::Vec3d(0.1, 0.2, 0.3), R2);
  cv::Vec3d t2(-0.3, -0.1, 0.2);

  cv::Vec3d v(0.5, 0.7, -1);                                    // a plane
  cv::Vec3d p_in_camera_coordinates(-1 - v(2), -1 - v(2), 1);   // a point in the plane

  EXPECT_NEAR(v.dot(p_in_camera_coordinates) + 1, 0.0, 1e-6);

  cv::Vec3d p = R1.t() * (p_in_camera_coordinates - t1);

  // Compute point projections in image 1 and 2.
  cv::Vec3d p1 = K1 * (R1 * p + t1);
  float x1 = p1(0) / p1(2), y1 = p1(1) / p1(2);
  cv::Vec3d p2 = K2 * (R2 * p + t2);
  float x2 = p2(0) / p2(2), y2 = p2(1) / p2(2);

  // Warp point via plane homography.
  cv::Matx33d H = PlaneInducedHomography(K1, R1, t1, K2, R2, t2, v);
  float x2_mapped, y2_mapped;
  ApplyHomography(H, x1, y1, &x2_mapped, &y2_mapped);

  EXPECT_NEAR(x2, x2_mapped, 1e-4);
  EXPECT_NEAR(y2, y2_mapped, 1e-4);
}


TEST(DepthOfPlaneBackprojection, DepthNormalPlaneLoop) {
  cv::Matx33d K(600, 0, 300, 0, 400, 200, 0, 0, 1);
  float depth = 3;
  cv::Vec3f normal(UniformRand(-1, 1), UniformRand(-1, 1), -1);
  cv::Vec3f plane = PlaneFromDepthAndNormal(20, 30, K, depth, normal);
  float backprojected_depth = DepthOfPlaneBackprojection(20, 30, K, plane);
  EXPECT_NEAR(depth, backprojected_depth, 1e-6);
}


TEST(Backproject, Reprojection) {
  cv::Matx33d K(600, 0, 300, 0, 400, 200, 0, 0, 1);
  cv::Matx33d R;
  cv::Rodrigues(cv::Vec3d(0.1, 0.1, 0.1), R);
  cv::Vec3d t(0.1, 0.2, 0.3);
  double u = 10, v = 20, depth = 13;
  cv::Vec3d x = Backproject(u, v, depth, K, R, t);
  cv::Vec3d reprojection = Project(x, K, R, t);
  EXPECT_NEAR(reprojection(2), depth, 1e-6);
  EXPECT_NEAR(reprojection(0) / reprojection(2), u, 1e-6);
  EXPECT_NEAR(reprojection(1) / reprojection(2), v, 1e-6);
}

TEST(NCCEstimator, Simple) {
  NCCEstimator ncc;
  ncc.Push(1, 1);
  ncc.Push(2, 2);
  ncc.Push(3, 3);
  EXPECT_NEAR(ncc.Get(), 1.0, 1e-6);
}

}  // namespace
