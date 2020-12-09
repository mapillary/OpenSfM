#include <geometry/camera_functions.h>


int GetIntValue(const double& x){
  return int(x);
}

int GetIntValue(const Eigen::AutoDiffScalar<Eigen::VectorXd>& x){
  return int(x.value());
}