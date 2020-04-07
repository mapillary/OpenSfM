#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sfm/tracks_manager.h>

namespace {

class TempFile {
 public:
  TempFile() {
    char tmpname[L_tmpnam];
    auto dummy = tmpnam(tmpname);
    filename = std::string(tmpname);
  }

  ~TempFile() { remove(filename.c_str()); }

  std::string Name() const { return filename; }

 private:
  std::string filename;
};

class TracksManagerTest : public ::testing::Test {
 protected:
  void SetUp() {
    track["1"] = Keypoint(1.0, 1.0, 1.0, 1, 1, 1, 1);
    track["2"] = Keypoint(2.0, 2.0, 2.0, 2, 2, 2, 2);
    track["3"] = Keypoint(3.0, 3.0, 3.0, 3, 3, 3, 3);
    manager.AddTrack("1", track);
  }

  TempFile tmpfile;
  TracksManager manager;
  std::unordered_map<ShotId, Keypoint> track;
};

TEST_F(TracksManagerTest, ReturnsShotsIDs) {
  EXPECT_THAT(manager.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1", "2", "3")));
}

TEST_F(TracksManagerTest, ReturnsTracksIDs) {
  EXPECT_THAT(manager.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));
}

TEST_F(TracksManagerTest, ReturnsObservation) {
  EXPECT_EQ(manager.GetObservation("1", "1"), Keypoint(1.0, 1.0, 1.0, 1, 1, 1, 1));
}

TEST_F(TracksManagerTest, ReturnsAllCommonObservations) {
  const auto tuple =
      std::make_tuple("1", Keypoint(1.0, 1.0, 1.0, 1, 1, 1, 1), Keypoint(2.0, 2.0, 2.0, 2, 2, 2, 2));
  std::vector<std::tuple<TrackId, Keypoint, Keypoint> > one_tuple{tuple};
  EXPECT_EQ(manager.GetAllCommonObservations("1", "2"), one_tuple);
}

TEST_F(TracksManagerTest, ReturnsObservationsOfPoint) {
  EXPECT_EQ(manager.GetObservationsOfPoint("1"), track);
}

TEST_F(TracksManagerTest, ReturnsObservationsOfShot) {
  std::unordered_map<TrackId, Keypoint> shot;
  shot["1"] = Keypoint(1.0, 1.0, 1.0, 1, 1, 1, 1);
  EXPECT_EQ(manager.GetObservationsOfShot("1"), shot);
}

TEST_F(TracksManagerTest, ReturnsObservationsOfPointsAtShot) {
  std::unordered_map<TrackId, Keypoint> shot;
  shot["1"] = Keypoint(1.0, 1.0, 1.0, 1, 1, 1, 1);
  EXPECT_EQ(manager.GetObservationsOfPointsAtShot({"1"}, "1"), shot);
}

TEST_F(TracksManagerTest, HasIOConsistency) {
  TracksManager::WriteToFile(tmpfile.Name(), manager);
  const TracksManager manager_new =
      TracksManager::InstanciateFromFile(tmpfile.Name());

  EXPECT_THAT(manager_new.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1", "2", "3")));
  EXPECT_THAT(manager_new.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));
  EXPECT_EQ(track, manager_new.GetObservationsOfPoint("1"));
}

}  // namespace
