#include <geo/geo.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(Geo, EcefLlaConsistency) {
  const Vec3d lla_bef(46.5274109, 6.5722075, 402.16);
  const Vec3d ecef = geo::EcefFromLla(lla_bef);
  const Vec3d lla_aft = geo::LlaFromEcef(ecef);
  ASSERT_TRUE(lla_bef.isApprox(lla_aft, 1e-3));
  const Vec3d ecef_2 = geo::EcefFromLla(lla_bef[0], lla_bef[1], lla_bef[2]);
  ASSERT_TRUE(ecef.isApprox(ecef_2, 1e-5));
  const Vec3d lla_aft_2 = geo::LlaFromEcef(ecef_2[0], ecef_2[1], ecef_2[2]);
  ASSERT_TRUE(lla_aft.isApprox(lla_aft_2, 1e-3));
}

TEST(Geo, EcefLlaTopocentricConsistency) {
  const Vec3d lla_ref(46.5, 6.5, 400);
  const Vec3d lla_bef(46.5274109, 6.5722075, 402.16);
  const Vec3d topo = geo::TopocentricFromLla(
      lla_bef[0], lla_bef[1], lla_bef[2], lla_ref[0], lla_ref[1], lla_ref[2]);
  const Vec3d topo2 = geo::TopocentricFromLla(lla_bef, lla_ref);
  ASSERT_TRUE(topo.isApprox(topo2, 1e-5));
  const Vec3d lla_after = geo::LlaFromTopocentric(topo, lla_ref);
  const Vec3d lla_after2 = geo::LlaFromTopocentric(
      topo[0], topo[1], topo[2], lla_ref[0], lla_ref[1], lla_ref[2]);
  ASSERT_TRUE(lla_after.isApprox(lla_after2, 1e-3));
  ASSERT_TRUE(lla_after.isApprox(lla_bef, 1e-3));
}

TEST(Geo, EcefFromTopocentricTransform) {
  const Vec3d ecef(30, 20, 10);
  ASSERT_TRUE(geo::EcefFromTopocentricTransform(ecef).isApprox(
      geo::EcefFromTopocentricTransformFiniteDiff(ecef), 1e-5));
}

TEST(Geo, RadDegreeConversion) {
  ASSERT_NEAR(geo::ToDegrees(2 * 3.14159265358979323846), 360, 1e-2);
  ASSERT_NEAR(geo::ToRadians(360), (2 * 3.14159265358979323846), 1e-2);
  ASSERT_NEAR(geo::ToRadians(geo::ToDegrees(0.34)), 0.34, 1e-2);
}

TEST(Geo, TopocentricConverter) {
  geo::TopocentricConverter topo_default;
  ASSERT_EQ(topo_default.lat_, 0.0);
  ASSERT_EQ(topo_default.long_, 0.0);
  ASSERT_EQ(topo_default.alt_, 0.0);
  geo::TopocentricConverter topo_lla(1.0, 2.0, 3.0);
  ASSERT_EQ(topo_lla.lat_, 1.0);
  ASSERT_EQ(topo_lla.long_, 2.0);
  ASSERT_EQ(topo_lla.alt_, 3.0);
  geo::TopocentricConverter topo_lla_vec(Vec3d(1, 2, 3));
  ASSERT_EQ(topo_lla_vec.lat_, 1.0);
  ASSERT_EQ(topo_lla_vec.long_, 2.0);
  ASSERT_EQ(topo_lla_vec.alt_, 3.0);

  const Vec3d lla_ref(46.5, 6.5, 400);
  geo::TopocentricConverter topo_conv(lla_ref);
  const Vec3d lla_bef(46.5274109, 6.5722075, 402.16);
  const Vec3d topo = topo_conv.ToTopocentric(lla_bef);
  ASSERT_TRUE(topo_conv.ToTopocentric(lla_bef[0], lla_bef[1], lla_bef[2])
                  .isApprox(topo, 1e-4));
  ASSERT_TRUE(topo_conv.ToLla(topo).isApprox(lla_bef, 1e-4));
  ASSERT_TRUE(
      topo_conv.ToLla(topo[0], topo[1], topo[2]).isApprox(lla_bef, 1e-4));
}
