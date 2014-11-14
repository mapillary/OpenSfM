#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "bundle.h"


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  if (argc != 4) {
    std::cerr << "usage: simple_bundle_adjuster <tracks> <input reconstruction> <output reconstruction>\n";
    return 1;
  }

  BALProblem bal_problem;
  if (!bal_problem.LoadJson(argv[1], argv[2])) {
    std::cerr << "ERROR: unable to open files " << argv[1] << " and " << argv[2] << "\n";
    return 1;
  }

  const Observation* observations = bal_problem.observations();

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.

    ceres::CostFunction* cost_function = 
        new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 6, 3>(
            new SnavelyReprojectionError(observations[i].coordinates[0],
                                         observations[i].coordinates[1],
                                         observations[i].camera->parameters[0]));

    problem.AddResidualBlock(cost_function,
                             new TruncatedLoss(9.0),
                             observations[i].camera->parameters,
                             observations[i].shot->parameters,
                             observations[i].point->parameters);
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  //options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  bal_problem.SaveJson(argv[3]);
  return 0;
}

