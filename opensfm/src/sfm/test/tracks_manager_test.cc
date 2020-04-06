#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sfm/tracks_manager.h>

namespace cv{
bool operator==(const cv::KeyPoint& k1, const cv::KeyPoint& k2) {
  return k1.angle == k2.angle && k1.pt == k2.pt && k1.size == k2.size &&
         k1.class_id == k2.class_id;
}
}

namespace {

class TracksManagerTest : public ::testing::Test {
 protected:
  void SetUp() {
    track["1"] = cv::KeyPoint(1.0, 1.0, 1.0);
    track["2"] = cv::KeyPoint(2.0, 2.0, 2.0);
    track["3"] = cv::KeyPoint(3.0, 3.0, 3.0);
    manager.AddTrack(1, track);
  }

  TracksManager manager;
  std::unordered_map<ShotId, cv::KeyPoint> track;
};

TEST_F(TracksManagerTest, ReturnsShotsIDs) {
  EXPECT_THAT(manager.GetShotIds(),
              ::testing::WhenSorted(::testing::ElementsAre("1", "2", "3")));
}

TEST_F(TracksManagerTest, ReturnsTracksIDs) {
  EXPECT_THAT(manager.GetTrackIds(),
              ::testing::WhenSorted(::testing::ElementsAre(1)));
}

TEST_F(TracksManagerTest, ReturnsObservation) {
  EXPECT_EQ(manager.GetObservation("1", 1), cv::KeyPoint(1.0, 1.0, 1.0));
}

TEST_F(TracksManagerTest, ReturnsAllCommonObservations) {
  const auto pair = std::make_pair(cv::KeyPoint(1.0, 1.0, 1.0), cv::KeyPoint(2.0, 2.0, 2.0));
  std::vector< std::pair<cv::KeyPoint, cv::KeyPoint> > one_pair{pair};
  EXPECT_EQ(manager.GetAllCommonObservations("1", "2"), one_pair);
}

TEST_F(TracksManagerTest, ReturnsObservationsOfPoint) {
  EXPECT_EQ(manager.GetObservationsOfPoint(1), track);
}

TEST_F(TracksManagerTest, ReturnsObservationsOfShot) {
  std::unordered_map<TrackId, cv::KeyPoint> shot;
  shot[1] = cv::KeyPoint(1.0, 1.0, 1.0);
  EXPECT_EQ(manager.GetObservationsOfShot("1"), shot);
}

TEST_F(TracksManagerTest, ReturnsObservationsOfPointsAtShott) {
  std::unordered_map<TrackId, cv::KeyPoint> shot;
  shot[1] = cv::KeyPoint(1.0, 1.0, 1.0);
  EXPECT_EQ(manager.GetObservationsOfPointsAtShot({1}, "1"), shot);
}

}  // namespace
