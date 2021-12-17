#include <geometry/triangulation.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(Point, IsRefined) {
  MatX3d centers(2, 3);
  MatX3d bearings(2, 3);
  Vec3d point;

  centers.row(0) << 0., 0., 0.;
  centers.row(1) << 4., 4., 0.;
  bearings.row(0) << 1., 2., 0.;
  bearings.row(1) << -2., 0., 0.;
  point << 0., 6., 0.;
  bearings.row(0).normalize();
  bearings.row(1).normalize();
  const auto refined = geometry::PointRefinement(centers, bearings, point, 10);

  Vec3d expected;
  expected << 2., 4., 0.;
  for (int i = 0; i < 3; ++i) {
    ASSERT_NEAR(expected(i), refined(i), 1e-6);
  }
}
