#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
