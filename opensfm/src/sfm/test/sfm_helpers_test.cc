#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sfm/sfm_helpers.h>
#include <sfm/tracks_manager.h>

namespace {

class SfmHelpersTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto o1 = Observation(1.0, 1.0, 1.0, 1, 1, 1, 1, 1, 1);
    const auto o2 = Observation(2.0, 2.0, 2.0, 2, 2, 2, 2, 2, 2);
    const auto o3 = Observation(3.0, 3.0, 3.0, 3, 3, 3, 3);
    manager.AddObservation("1", "1", o1);
    manager.AddObservation("2", "1", o2);
    manager.AddObservation("3", "1", o3);
    track["1"] = o1;
    track["2"] = o2;
    track["3"] = o3;

    connections_add.push_back("1");
    connections_add.push_back("2");
    connections_add.push_back("3");

    connections_remove.push_back("1");
  }

  TracksManager manager;
  std::vector<TrackId> connections_add;
  std::vector<TrackId> connections_remove;
  std::unordered_map<ShotId, Observation> track;
};

TEST_F(SfmHelpersTest, CountTracksPerShot) {
  const auto counts =
      sfm_helpers::CountTracksPerShot(manager, {"1", "2", "3"}, {"1"});

  EXPECT_EQ(1, counts.at("1"));
  EXPECT_EQ(1, counts.at("2"));
  EXPECT_EQ(1, counts.at("3"));
}

TEST_F(SfmHelpersTest, AddConnections) {
  sfm_helpers::AddConnections(manager, "1", connections_add);
  sfm_helpers::AddConnections(manager, "2", connections_add);
  sfm_helpers::AddConnections(manager, "3", connections_add);

  const auto counts = sfm_helpers::CountTracksPerShot(manager, {"1", "2", "3"},
                                                      {"1", "2", "3"});

  EXPECT_EQ(3, counts.at("1"));
  EXPECT_EQ(3, counts.at("2"));
  EXPECT_EQ(3, counts.at("3"));
}

TEST_F(SfmHelpersTest, RemoveConnections) {
  sfm_helpers::RemoveConnections(manager, "1", connections_remove);

  const auto counts =
      sfm_helpers::CountTracksPerShot(manager, {"1", "2", "3"}, {"1"});

  EXPECT_EQ(0, counts.at("1"));
  EXPECT_EQ(1, counts.at("2"));
  EXPECT_EQ(1, counts.at("3"));
}
}  // namespace
