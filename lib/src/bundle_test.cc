
#include "gtest/gtest.h"
#include <glog/logging.h>
#include "bundle.h"


TEST(BALProblem, LoadJson) {
  BALProblem bal_problem;
  bool w = bal_problem.LoadJson("../../data/ET/tracks.csv",
                                "../../data/ET/reconstruction.json");
  EXPECT_TRUE(w);
  EXPECT_GE(bal_problem.num_observations(), 10);

  int outliers = 0;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    const Observation* observation = &bal_problem.observations()[i];

    SnavelyReprojectionError re(observation->coordinates[0],
                                observation->coordinates[1]);

    double residuals[2];
    re(observation->camera->parameters,
       observation->shot->parameters,
       observation->point->parameters,
       residuals);
    if (fabs(residuals[0]) > 2.0 || fabs(residuals[1]) > 2.0) {
      outliers++;
    }
  }
  EXPECT_LE(outliers, bal_problem.num_observations() / 2);
}


DEFINE_string(test_tmpdir, "/tmp", "Dir to use for temp files");

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}

