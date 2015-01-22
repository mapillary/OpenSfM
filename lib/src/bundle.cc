#include <cmath>
#include <cstdio>
#include <iostream>
#include <ctime>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "bundle.h"


DEFINE_double(exif_focal_sd, 0.01, "The standard deviation of the difference between the real focal and the EXIF focal");
DEFINE_string(input, "", "The initial reconstruction file");
DEFINE_string(output, "", "The bundle adjusted reconstruction file");
DEFINE_string(tracks, "", "The tracks.csv file with the correspondence information");
DEFINE_string(loss_function, "TruncatedLoss", "Loss function for the ceres problem (see: http://ceres-solver.org/modeling.html#lossfunction)");
DEFINE_double(loss_function_threshold, 3.0, "Usually cost is quadratic for smaller residuals and sub-quadratic above");


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!FLAGS_input.size() || !FLAGS_output.size() || !FLAGS_tracks.size() ) {
    std::cerr << "usage: simple_bundle_adjuster --tracks <tracks> --input <input reconstruction> --output <output reconstruction>\n";
    return 1;
  }

  clock_t start_reading = clock();

  BundleAdjuster ba;
  if (!ba.LoadJson(FLAGS_tracks.c_str(), FLAGS_input.c_str())) {
    std::cerr << "ERROR: unable to open files " << FLAGS_tracks << " and " << FLAGS_input << "\n";
    return 1;
  }
  ba.SetLossFunction(FLAGS_loss_function, FLAGS_loss_function_threshold);
  ba.SetFocalPriorSD(FLAGS_exif_focal_sd);
  
  clock_t start_solving = clock();

  ba.run();

  clock_t start_writing = clock();

  ba.SaveJson(FLAGS_output.c_str());

  clock_t finish = clock();
  std::cout << "Bundle done reading/solving/writing: "
            << ((float)start_solving - start_reading)/CLOCKS_PER_SEC << "/"
            << ((float)start_writing - start_solving)/CLOCKS_PER_SEC << "/"
            << ((float)finish - start_writing)/CLOCKS_PER_SEC << "\n";
  return 0;
}

