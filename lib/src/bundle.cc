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
  BALProblem bal_problem;
  if (!bal_problem.LoadJson(FLAGS_tracks.c_str(), FLAGS_input.c_str())) {
    std::cerr << "ERROR: unable to open files " << FLAGS_tracks << " and " << FLAGS_input << "\n";
    return 1;
  }
  clock_t start_solving = clock();

  ceres::LossFunction *loss;
  if (FLAGS_loss_function.compare("TruncatedLoss") == 0) {
    loss = new TruncatedLoss(FLAGS_loss_function_threshold);
  } else if (FLAGS_loss_function.compare("TrivialLoss") == 0) {
    loss = new ceres::TrivialLoss();
  } else if (FLAGS_loss_function.compare("HuberLoss") == 0) {
    loss = new ceres::HuberLoss(FLAGS_loss_function_threshold);
  } else if (FLAGS_loss_function.compare("SoftLOneLoss") == 0) {
    loss = new ceres::SoftLOneLoss(FLAGS_loss_function_threshold);
  } else if (FLAGS_loss_function.compare("CauchyLoss") == 0) {
    loss = new ceres::CauchyLoss(FLAGS_loss_function_threshold);
  } else if (FLAGS_loss_function.compare("ArctanLoss") == 0) {
    loss = new ceres::ArctanLoss(FLAGS_loss_function_threshold);
  }

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;

  const Observation* observations = bal_problem.observations();
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.

    ceres::CostFunction* cost_function = 
        new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 6, 3>(
            new SnavelyReprojectionError(observations[i].coordinates[0],
                                         observations[i].coordinates[1]));

    problem.AddResidualBlock(cost_function,
                             loss,
                             observations[i].camera->parameters,
                             observations[i].shot->parameters,
                             observations[i].point->parameters);
  }

  Camera* cameras = bal_problem.cameras();
  for (int i = 0; i < bal_problem.num_cameras(); ++i) {
    double exif_focal_sd_in_pixels = FLAGS_exif_focal_sd * cameras[i].width;
    ceres::CostFunction* cost_function = 
        new ceres::AutoDiffCostFunction<FocalPriorError, 1, 3>(
            new FocalPriorError(cameras[i].exif_focal, exif_focal_sd_in_pixels));

    problem.AddResidualBlock(cost_function,
                             NULL,
                             cameras[i].parameters);
  }

  // Shot* shots = bal_problem.shots();
  // for (int i = 0; i < bal_problem.num_shots(); ++i) {
  //   ceres::CostFunction* cost_function = 
  //       new ceres::AutoDiffCostFunction<GPSPriorError, 3, 6>(
  //           new GPSPriorError(shots[i].gps_position[0],
  //                             shots[i].gps_position[1],
  //                             shots[i].gps_position[2],
  //                             shots[i].gps_dop));

  //   problem.AddResidualBlock(cost_function,
  //                            NULL,
  //                            shots[i].parameters);
  // }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  clock_t start_writing = clock();


  bal_problem.SaveJson(FLAGS_output.c_str());
  clock_t finish = clock();
  std::cout << "Bundle done reading/solving/writing: "
            << ((float)start_solving - start_reading)/CLOCKS_PER_SEC << "/"
            << ((float)start_writing - start_solving)/CLOCKS_PER_SEC << "/"
            << ((float)finish - start_writing)/CLOCKS_PER_SEC << "\n";
  return 0;
}

