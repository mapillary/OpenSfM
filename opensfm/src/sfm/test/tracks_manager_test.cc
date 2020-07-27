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
    const auto o1 =  Observation(1.0, 1.0, 1.0, 1, 1, 1, 1);
    const auto o2 =  Observation(2.0, 2.0, 2.0, 2, 2, 2, 2);
    const auto o3 =  Observation(3.0, 3.0, 3.0, 3, 3, 3, 3);
    manager.AddObservation("1", "1", o1);
    manager.AddObservation("2", "1", o2);
    manager.AddObservation("3", "1", o3);
    track["1"] = o1;
    track["2"] = o2;
    track["3"] = o3;
  }

  TempFile tmpfile;
  TracksManager manager;
  std::unordered_map<ShotId, Observation> track;
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
  EXPECT_EQ(manager.GetObservation("1", "1"), Observation(1.0, 1.0, 1.0, 1, 1, 1, 1));
}

TEST_F(TracksManagerTest, AddsObservation) {
  Observation obs(4.0, 4.0, 4.0, 4, 4, 4, 4);
  manager.AddObservation("4", "1", obs);
  EXPECT_EQ(manager.GetObservation("4", "1"), obs);
}

TEST_F(TracksManagerTest, RemoveObservation) {
  manager.RemoveObservation("3", "1");
  auto copy = track;
  copy.erase("3");
  EXPECT_EQ(manager.GetTrackObservations("1"), copy);
}

TEST_F(TracksManagerTest, ReturnsAllCommonObservations) {
  const auto tuple =
      std::make_tuple("1", Observation(1.0, 1.0, 1.0, 1, 1, 1, 1), Observation(2.0, 2.0, 2.0, 2, 2, 2, 2));
  std::vector<std::tuple<TrackId, Observation, Observation> > one_tuple{tuple};
  EXPECT_EQ(manager.GetAllCommonObservations("1", "2"), one_tuple);
}

TEST_F(TracksManagerTest, ReturnsTrackObservations) {
  EXPECT_EQ(manager.GetTrackObservations("1"), track);
}

TEST_F(TracksManagerTest, ReturnsShotObservations) {
  std::unordered_map<TrackId, Observation> shot;
  shot["1"] = Observation(1.0, 1.0, 1.0, 1, 1, 1, 1);
  EXPECT_EQ(manager.GetShotObservations("1"), shot);
}

TEST_F(TracksManagerTest, ConstructSubTracksManager) {
  const auto subset = manager.ConstructSubTracksManager({"1"}, {"2", "3"});
  EXPECT_THAT(subset.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("2", "3")));
  EXPECT_THAT(subset.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));

  std::unordered_map<ShotId, Observation> subtrack;
  subtrack["2"] = Observation(2.0, 2.0, 2.0, 2, 2, 2, 2);
  subtrack["3"] = Observation(3.0, 3.0, 3.0, 3, 3, 3, 3);
  EXPECT_EQ(subtrack, subset.GetTrackObservations("1"));
}

TEST_F(TracksManagerTest, HasIOFileConsistency) {
  manager.WriteToFile(tmpfile.Name());
  const TracksManager manager_new =
      TracksManager::InstanciateFromFile(tmpfile.Name());

  EXPECT_THAT(manager_new.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1", "2", "3")));
  EXPECT_THAT(manager_new.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));
  EXPECT_EQ(track, manager_new.GetTrackObservations("1"));
}

TEST_F(TracksManagerTest, HasIOStringConsistency) {
  const auto serialized = manager.AsSring();
  const TracksManager manager_new =
      TracksManager::InstanciateFromString(serialized);

  EXPECT_THAT(manager_new.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1", "2", "3")));
  EXPECT_THAT(manager_new.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));
  EXPECT_EQ(track, manager_new.GetTrackObservations("1"));
}

}  // namespace
