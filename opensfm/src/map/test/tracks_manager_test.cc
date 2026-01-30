#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <map/tracks_manager.h>

namespace {

class TempFile {
 public:
  TempFile() {
    char filenameTmp[] = "/tmp/opensfm_tracks_manager_test_XXXXXX";
    int fd = mkstemp(filenameTmp);
    if (fd == -1) {
      std::runtime_error("Could not create temporary file");
    }
    filename = std::string(filenameTmp);
  }

  ~TempFile() { remove(filename.c_str()); }

  std::string Name() const { return filename; }

 private:
  std::string filename;
};

class TracksManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto o1 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 1, 1, 1);
    const auto o2 = map::Observation(2.0, 2.0, 2.0, 2, 2, 2, 2, 2, 2);
    const auto o3 = map::Observation(3.0, 3.0, 3.0, 3, 3, 3, 3);
    manager.AddObservation("1", "1", o1);
    manager.AddObservation("2", "1", o2);
    manager.AddObservation("3", "1", o3);
    track["1"] = o1;
    track["2"] = o2;
    track["3"] = o3;
  }

  TempFile tmpfile;
  map::TracksManager manager;
  std::unordered_map<map::ShotId, map::Observation> track;
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
  EXPECT_EQ(manager.GetObservation("1", "1"),
            map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 1, 1, 1));
}

TEST_F(TracksManagerTest, AddsObservation) {
  map::Observation obs(4.0, 4.0, 4.0, 4, 4, 4, 4);
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
      std::make_tuple("1", map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 1, 1, 1),
                      map::Observation(2.0, 2.0, 2.0, 2, 2, 2, 2, 2, 2));
  std::vector<std::tuple<map::TrackId, map::Observation, map::Observation> >
      one_tuple{tuple};
  EXPECT_EQ(manager.GetAllCommonObservations("1", "2"), one_tuple);
}

TEST_F(TracksManagerTest, ReturnsTrackObservations) {
  EXPECT_EQ(manager.GetTrackObservations("1"), track);
}

TEST_F(TracksManagerTest, ReturnsShotObservations) {
  std::unordered_map<map::TrackId, map::Observation> shot;
  shot["1"] = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 1, 1, 1);
  EXPECT_EQ(manager.GetShotObservations("1"), shot);
}

TEST_F(TracksManagerTest, ConstructSubTracksManager) {
  const auto subset = manager.ConstructSubTracksManager({"1"}, {"2", "3"});
  EXPECT_THAT(subset.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("2", "3")));
  EXPECT_THAT(subset.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));

  std::unordered_map<map::ShotId, map::Observation> subtrack;
  subtrack["2"] = map::Observation(2.0, 2.0, 2.0, 2, 2, 2, 2, 2, 2);
  subtrack["3"] = map::Observation(3.0, 3.0, 3.0, 3, 3, 3, 3);
  EXPECT_EQ(subtrack, subset.GetTrackObservations("1"));
}

TEST_F(TracksManagerTest, MergeThreeTracksManager) {
  map::TracksManager manager1;
  const auto o0 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 0);
  const auto o1 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 1);
  const auto o2 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 2);
  const auto o3 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 3);
  manager1.AddObservation("1", "0", o0);
  manager1.AddObservation("1", "1", o1);
  manager1.AddObservation("2", "1", o2);
  manager1.AddObservation("3", "1", o3);

  map::TracksManager manager2;
  const auto o4 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 4);
  const auto o5 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 5);
  const auto o6 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 6);
  manager2.AddObservation("3", "1", o3);
  manager2.AddObservation("4", "1", o4);
  manager2.AddObservation("5", "1", o5);
  manager2.AddObservation("6", "2", o6);

  map::TracksManager manager3;
  const auto o7 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 7);
  const auto o8 = map::Observation(1.0, 1.0, 1.0, 1, 1, 1, 8);
  manager3.AddObservation("6", "1", o6);
  manager3.AddObservation("7", "1", o7);
  manager3.AddObservation("8", "2", o8);

  auto merged =
      map::TracksManager::MergeTracksManager({&manager1, &manager2, &manager3});

  const auto track0 = merged.GetTrackObservations("0");
  const auto track1 = merged.GetTrackObservations("1");
  const auto track2 = merged.GetTrackObservations("2");
  const auto track3 = merged.GetTrackObservations("3");

  EXPECT_THAT(
      merged.GetTrackIds(),
      ::testing::WhenSorted(::testing::ElementsAre("0", "1", "2", "3")));

  std::unordered_map<map::ShotId, map::Observation> gt_trackA;
  gt_trackA["1"] = o1;
  gt_trackA["2"] = o2;
  gt_trackA["3"] = o3;
  gt_trackA["4"] = o4;
  gt_trackA["5"] = o5;

  std::unordered_map<map::ShotId, map::Observation> gt_trackB;
  gt_trackB["6"] = o6;
  gt_trackB["7"] = o7;

  std::unordered_map<map::ShotId, map::Observation> gt_trackC;
  gt_trackC["8"] = o8;

  std::unordered_map<map::ShotId, map::Observation> gt_trackD;
  gt_trackD["1"] = o0;

  // Expect the two sets of tracks to be the same for some reordering
  EXPECT_TRUE(track0 == gt_trackA || track1 == gt_trackA ||
              track2 == gt_trackA || track3 == gt_trackA);
  EXPECT_TRUE(track0 == gt_trackB || track1 == gt_trackB ||
              track2 == gt_trackB || track3 == gt_trackB);
  EXPECT_TRUE(track0 == gt_trackC || track1 == gt_trackC ||
              track2 == gt_trackC || track3 == gt_trackC);
  EXPECT_TRUE(track0 == gt_trackD || track1 == gt_trackD ||
              track2 == gt_trackD || track3 == gt_trackD);
}

TEST_F(TracksManagerTest, HasIOFileConsistency) {
  manager.WriteToFile(tmpfile.Name());
  const map::TracksManager manager_new =
      map::TracksManager::InstanciateFromFile(tmpfile.Name());

  EXPECT_THAT(manager_new.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1", "2", "3")));
  EXPECT_THAT(manager_new.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));
  EXPECT_EQ(track, manager_new.GetTrackObservations("1"));
}

TEST_F(TracksManagerTest, HasIOStringConsistency) {
  const auto serialized = manager.AsString();
  const map::TracksManager manager_new =
      map::TracksManager::InstanciateFromString(serialized);

  EXPECT_THAT(manager_new.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1", "2", "3")));
  EXPECT_THAT(manager_new.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1")));
  EXPECT_EQ(track, manager_new.GetTrackObservations("1"));
}

}  // namespace
