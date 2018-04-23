#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <map>
#include <string>

extern "C" {
#include <string.h>
}

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"
#include "ceres/covariance.h"



enum BACameraType {
  BA_PERSPECTIVE_CAMERA,
  BA_BROWN_PERSPECTIVE_CAMERA,
  BA_FISHEYE_CAMERA,
  BA_EQUIRECTANGULAR_CAMERA
};

struct BACamera {
  std::string id;
  bool constant;

  virtual BACameraType type() = 0;
};

enum {
  BA_CAMERA_FOCAL,
  BA_CAMERA_K1,
  BA_CAMERA_K2,
  BA_CAMERA_NUM_PARAMS
};

enum {
  BA_BROWN_CAMERA_FOCAL_X,
  BA_BROWN_CAMERA_FOCAL_Y,
  BA_BROWN_CAMERA_C_X,
  BA_BROWN_CAMERA_C_Y,
  BA_BROWN_CAMERA_K1,
  BA_BROWN_CAMERA_K2,
  BA_BROWN_CAMERA_P1,
  BA_BROWN_CAMERA_P2,
  BA_BROWN_CAMERA_K3,
  BA_BROWN_CAMERA_NUM_PARAMS
};

struct BAPerspectiveCamera : public BACamera {
  double parameters[BA_CAMERA_NUM_PARAMS];
  double focal_prior;
  double k1_prior;
  double k2_prior;

  BACameraType type() { return BA_PERSPECTIVE_CAMERA; }
  double GetFocal() { return parameters[BA_CAMERA_FOCAL]; }
  double GetK1() { return parameters[BA_CAMERA_K1]; }
  double GetK2() { return parameters[BA_CAMERA_K2]; }
  void SetFocal(double v) { parameters[BA_CAMERA_FOCAL] = v; }
  void SetK1(double v) { parameters[BA_CAMERA_K1] = v; }
  void SetK2(double v) { parameters[BA_CAMERA_K2] = v; }
};

struct BABrownPerspectiveCamera : public BACamera {
  double parameters[BA_BROWN_CAMERA_NUM_PARAMS];
  double focal_x_prior;
  double focal_y_prior;
  double c_x_prior;
  double c_y_prior;
  double k1_prior;
  double k2_prior;
  double p1_prior;
  double p2_prior;
  double k3_prior;

  BACameraType type() { return BA_BROWN_PERSPECTIVE_CAMERA; }
  double GetFocalX() { return parameters[BA_BROWN_CAMERA_FOCAL_X]; }
  double GetFocalY() { return parameters[BA_BROWN_CAMERA_FOCAL_Y]; }
  double GetCX() { return parameters[BA_BROWN_CAMERA_C_X]; }
  double GetCY() { return parameters[BA_BROWN_CAMERA_C_Y]; }
  double GetK1() { return parameters[BA_BROWN_CAMERA_K1]; }
  double GetK2() { return parameters[BA_BROWN_CAMERA_K2]; }
  double GetP1() { return parameters[BA_BROWN_CAMERA_P1]; }
  double GetP2() { return parameters[BA_BROWN_CAMERA_P2]; }
  double GetK3() { return parameters[BA_BROWN_CAMERA_K3]; }
  void SetFocalX(double v) { parameters[BA_BROWN_CAMERA_FOCAL_X] = v; }
  void SetFocalY(double v) { parameters[BA_BROWN_CAMERA_FOCAL_Y] = v; }
  void SetCX(double v) { parameters[BA_BROWN_CAMERA_C_X] = v; }
  void SetCY(double v) { parameters[BA_BROWN_CAMERA_C_Y] = v; }
  void SetK1(double v) { parameters[BA_BROWN_CAMERA_K1] = v; }
  void SetK2(double v) { parameters[BA_BROWN_CAMERA_K2] = v; }
  void SetP1(double v) { parameters[BA_BROWN_CAMERA_P1] = v; }
  void SetP2(double v) { parameters[BA_BROWN_CAMERA_P2] = v; }
  void SetK3(double v) { parameters[BA_BROWN_CAMERA_K3] = v; }
};


struct BAFisheyeCamera : public BACamera{
  double parameters[BA_CAMERA_NUM_PARAMS];
  double focal_prior;
  double k1_prior;
  double k2_prior;

  BACameraType type() { return BA_FISHEYE_CAMERA; }
  double GetFocal() { return parameters[BA_CAMERA_FOCAL]; }
  double GetK1() { return parameters[BA_CAMERA_K1]; }
  double GetK2() { return parameters[BA_CAMERA_K2]; }
  void SetFocal(double v) { parameters[BA_CAMERA_FOCAL] = v; }
  void SetK1(double v) { parameters[BA_CAMERA_K1] = v; }
  void SetK2(double v) { parameters[BA_CAMERA_K2] = v; }
};

struct BAEquirectangularCamera : public BACamera {
  BACameraType type() { return BA_EQUIRECTANGULAR_CAMERA; }
};

enum {
  BA_SHOT_RX,
  BA_SHOT_RY,
  BA_SHOT_RZ,
  BA_SHOT_TX,
  BA_SHOT_TY,
  BA_SHOT_TZ,
  BA_SHOT_NUM_PARAMS
};

struct BAShot {
  double parameters[BA_SHOT_NUM_PARAMS];
  double covariance[BA_SHOT_NUM_PARAMS * BA_SHOT_NUM_PARAMS];
  bool constant;
  int exif_orientation;
  std::string camera;
  std::string id;

  double GetRX() { return parameters[BA_SHOT_RX]; }
  double GetRY() { return parameters[BA_SHOT_RY]; }
  double GetRZ() { return parameters[BA_SHOT_RZ]; }
  double GetTX() { return parameters[BA_SHOT_TX]; }
  double GetTY() { return parameters[BA_SHOT_TY]; }
  double GetTZ() { return parameters[BA_SHOT_TZ]; }
  void SetRX(double v) { parameters[BA_SHOT_RX] = v; }
  void SetRY(double v) { parameters[BA_SHOT_RY] = v; }
  void SetRZ(double v) { parameters[BA_SHOT_RZ] = v; }
  void SetTX(double v) { parameters[BA_SHOT_TX] = v; }
  void SetTY(double v) { parameters[BA_SHOT_TY] = v; }
  void SetTZ(double v) { parameters[BA_SHOT_TZ] = v; }
  double GetCovariance(int i, int j) { return covariance[i * BA_SHOT_NUM_PARAMS + j]; }
};

struct BAPoint {
  double coordinates[3];
  bool constant;
  double reprojection_error;
  std::string id;

  double GetX() { return coordinates[0]; }
  double GetY() { return coordinates[1]; }
  double GetZ() { return coordinates[2]; }
  void SetX(double v) { coordinates[0] = v; }
  void SetY(double v) { coordinates[1] = v; }
  void SetZ(double v) { coordinates[2] = v; }
};

struct BAObservation {
  double coordinates[2];
  BACamera *camera;
  BAShot *shot;
  BAPoint *point;
};

struct BARotationPrior {
  BAShot *shot;
  double rotation[3];
  double std_deviation;
};

struct BATranslationPrior {
  BAShot *shot;
  double translation[3];
  double std_deviation;
};

struct BAPositionPrior {
  BAShot *shot;
  double position[3];
  double std_deviation;
};

struct BAPointPositionPrior {
  BAPoint *point;
  double position[3];
  double std_deviation;
};

struct BAGroundControlPointObservation {
  BACamera *camera;
  BAShot *shot;
  double coordinates3d[3];
  double coordinates2d[2];
};

class TruncatedLoss : public ceres::LossFunction {
 public:
  explicit TruncatedLoss(double t)
    : t2_(t*t) {
    CHECK_GT(t, 0.0);
  }

  virtual void Evaluate(double s, double rho[3]) const {
    if (s >= t2_) {
      // Outlier.
      rho[0] = t2_;
      rho[1] = std::numeric_limits<double>::min();
      rho[2] = 0.0;
    } else {
      // Inlier.
      rho[0] = s;
      rho[1] = 1.0;
      rho[2] = 0.0;
    }
  }

 private:
  const double t2_;
};


template <typename T>
void WorldToCameraCoordinates(const T* const shot,
                              const T world_point[3],
                              T camera_point[3]) {
  ceres::AngleAxisRotatePoint(shot + BA_SHOT_RX, world_point, camera_point);
  camera_point[0] += shot[BA_SHOT_TX];
  camera_point[1] += shot[BA_SHOT_TY];
  camera_point[2] += shot[BA_SHOT_TZ];
}


template <typename T>
void PerspectiveProject(const T* const camera,
                        const T point[3],
                        T projection[2]) {
  T xp = point[0] / point[2];
  T yp = point[1] / point[2];

  // Apply second and fourth order radial distortion.
  const T& l1 = camera[BA_CAMERA_K1];
  const T& l2 = camera[BA_CAMERA_K2];
  T r2 = xp * xp + yp * yp;
  T distortion = T(1.0) + r2  * (l1 + r2 * l2);

  // Compute final projected point position.
  const T& focal = camera[BA_CAMERA_FOCAL];
  projection[0] = focal * distortion * xp;
  projection[1] = focal * distortion * yp;
}


struct PerspectiveReprojectionError {
  PerspectiveReprojectionError(double observed_x, double observed_y, double std_deviation)
      : observed_x_(observed_x)
      , observed_y_(observed_y)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const shot,
                  const T* const point,
                  T* residuals) const {
    T camera_point[3];
    WorldToCameraCoordinates(shot, point, camera_point);

    if (camera_point[2] <= T(0.0)) {
      residuals[0] = residuals[1] = T(99.0);
      return true;
    }

    T predicted[2];
    PerspectiveProject(camera, camera_point, predicted);

    // The error is the difference between the predicted and observed position.
    residuals[0] = T(scale_) * (predicted[0] - T(observed_x_));
    residuals[1] = T(scale_) * (predicted[1] - T(observed_y_));

    return true;
  }

  double observed_x_;
  double observed_y_;
  double scale_;
};

template <typename T>
void BrownPerspectiveProject(const T* const camera,
                             const T point[3],
                             T projection[2]) {
  T xp = point[0] / point[2];
  T yp = point[1] / point[2];

  // Apply Brown--Conrady radial and tangential distortion
  const T& k1 = camera[BA_BROWN_CAMERA_K1];
  const T& k2 = camera[BA_BROWN_CAMERA_K2];
  const T& p1 = camera[BA_BROWN_CAMERA_P1];
  const T& p2 = camera[BA_BROWN_CAMERA_P2];
  const T& k3 = camera[BA_BROWN_CAMERA_K3];
  T r2 = xp * xp + yp * yp;
  T radial_distortion = T(1.0) + r2  * (k1 + r2 * (k2 + r2 * k3));

  T x_tangential_distortion = T(2.0) * p1 * xp * yp + p2 * (r2 + T(2.0) * xp * xp);
  T x_distorted = xp * radial_distortion + x_tangential_distortion;

  T y_tangential_distortion = p1 * (r2 + T(2.0) * yp * yp) + T(2.0) * p2 * xp * yp;
  T y_distorted = yp * radial_distortion + y_tangential_distortion;

  // Compute final projected point position.
  const T& focal_x = camera[BA_BROWN_CAMERA_FOCAL_X];
  const T& focal_y = camera[BA_BROWN_CAMERA_FOCAL_Y];
  const T& c_x = camera[BA_BROWN_CAMERA_C_X];
  const T& c_y = camera[BA_BROWN_CAMERA_C_Y];
  projection[0] = focal_x * x_distorted + c_x;
  projection[1] = focal_y * y_distorted + c_y;
}

struct BrownPerspectiveReprojectionError {
  BrownPerspectiveReprojectionError(double observed_x, double observed_y, double std_deviation)
      : observed_x_(observed_x)
      , observed_y_(observed_y)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const shot,
                  const T* const point,
                  T* residuals) const {
    T camera_point[3];
    WorldToCameraCoordinates(shot, point, camera_point);

    if (camera_point[2] <= T(0.0)) {
      residuals[0] = residuals[1] = T(99.0);
      return true;
    }

    T predicted[2];
    BrownPerspectiveProject(camera, camera_point, predicted);

    // The error is the difference between the predicted and observed position.
    residuals[0] = T(scale_) * (predicted[0] - T(observed_x_));
    residuals[1] = T(scale_) * (predicted[1] - T(observed_y_));

    return true;
  }

  double observed_x_;
  double observed_y_;
  double scale_;
};

template <typename T>
void FisheyeProject(const T* const camera,
                    const T point[3],
                    T projection[2]) {
  const T& focal = camera[BA_CAMERA_FOCAL];
  const T& k1 = camera[BA_CAMERA_K1];
  const T& k2 = camera[BA_CAMERA_K2];
  const T &x = point[0];
  const T &y = point[1];
  const T &z = point[2];

  T l = sqrt(x * x + y * y);
  T theta = atan2(l, z);
  T theta2 = theta * theta;
  T theta_d = theta * (T(1.0) + theta2 * (k1 + theta2 * k2));
  T s = focal * theta_d / l;

  projection[0] = s * x;
  projection[1] = s * y;
}


struct FisheyeReprojectionError {
  FisheyeReprojectionError(double observed_x, double observed_y, double std_deviation)
      : observed_x_(observed_x)
      , observed_y_(observed_y)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const shot,
                  const T* const point,
                  T* residuals) const {
    T camera_point[3];
    WorldToCameraCoordinates(shot, point, camera_point);

    if (camera_point[2] <= T(0.0)) {
      residuals[0] = residuals[1] = T(99.0);
      return true;
    }

    T predicted[2];
    FisheyeProject(camera, camera_point, predicted);

    // The error is the difference between the predicted and observed position.
    residuals[0] = T(scale_) * (predicted[0] - T(observed_x_));
    residuals[1] = T(scale_) * (predicted[1] - T(observed_y_));

    return true;
  }

  double observed_x_;
  double observed_y_;
  double scale_;
};

struct EquirectangularReprojectionError {
  EquirectangularReprojectionError(double observed_x, double observed_y, double std_deviation)
      : scale_(1.0 / std_deviation)
  {
    double lon = observed_x * 2 * M_PI;
    double lat = -observed_y * 2 * M_PI;
    bearing_vector_[0] = cos(lat) * sin(lon);
    bearing_vector_[1] = -sin(lat);
    bearing_vector_[2] = cos(lat) * cos(lon);
  }

  template <typename T>
  bool operator()(const T* const shot,
                  const T* const point,
                  T* residuals) const {
    // Position vector in camera coordinates.
    T p[3];
    WorldToCameraCoordinates(shot, point, p);

    // Project to unit sphere.
    const T l = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    p[0] /= l;
    p[1] /= l;
    p[2] /= l;

    // Difference between projected vector and observed bearing vector
    // We use the difference between unit vectors as an approximation
    // to the angle for small angles.
    residuals[0] = T(scale_) * (p[0] - T(bearing_vector_[0]));
    residuals[1] = T(scale_) * (p[1] - T(bearing_vector_[1]));
    residuals[2] = T(scale_) * (p[2] - T(bearing_vector_[2]));

    return true;
  }

  double bearing_vector_[3];
  double scale_;
};


struct GCPPerspectiveProjectionError {
  GCPPerspectiveProjectionError(
    double world[3], double observed[2], double std_deviation)
      : world_(world)
      , observed_(observed)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const shot,
                  T* residuals) const {
    T world_point[3] = { T(world_[0]), T(world_[1]), T(world_[2]) };
    T camera_point[3];
    WorldToCameraCoordinates(shot, world_point, camera_point);

    if (camera_point[2] <= T(0.0)) {
      residuals[0] = residuals[1] = T(99.0);
      return true;
    }

    T predicted[2];
    PerspectiveProject(camera, camera_point, predicted);

    // The error is the difference between the predicted and observed position.
    residuals[0] = T(scale_) * (predicted[0] - T(observed_[0]));
    residuals[1] = T(scale_) * (predicted[1] - T(observed_[1]));

    return true;
  }

  double *world_;
  double *observed_;
  double scale_;
};

struct GCPBrownPerspectiveProjectionError {
  GCPBrownPerspectiveProjectionError(
    double world[3], double observed[2], double std_deviation)
      : world_(world)
      , observed_(observed)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const shot,
                  T* residuals) const {
    T world_point[3] = { T(world_[0]), T(world_[1]), T(world_[2]) };
    T camera_point[3];
    WorldToCameraCoordinates(shot, world_point, camera_point);

    if (camera_point[2] <= T(0.0)) {
      residuals[0] = residuals[1] = T(99.0);
      return true;
    }

    T predicted[2];
    BrownPerspectiveProject(camera, camera_point, predicted);

    // The error is the difference between the predicted and observed position.
    residuals[0] = T(scale_) * (predicted[0] - T(observed_[0]));
    residuals[1] = T(scale_) * (predicted[1] - T(observed_[1]));

    return true;
  }

  double *world_;
  double *observed_;
  double scale_;
};

struct GCPEquirectangularProjectionError {
  GCPEquirectangularProjectionError(
    double world[3], double observed[2], double std_deviation)
      : world_(world)
      , scale_(1.0 / std_deviation)
  {
    double lon = observed[0] * 2 * M_PI;
    double lat = -observed[1] * 2 * M_PI;
    bearing_vector_[0] = cos(lat) * sin(lon);
    bearing_vector_[1] = -sin(lat);
    bearing_vector_[2] = cos(lat) * cos(lon);
  }

  template <typename T>
  bool operator()(const T* const shot,
                  T* residuals) const {
    // Position vector in camera coordinates.
    T world_point[3] = { T(world_[0]), T(world_[1]), T(world_[2]) };
    T p[3];
    WorldToCameraCoordinates(shot, world_point, p);

    // Project to unit sphere.
    const T l = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    p[0] /= l;
    p[1] /= l;
    p[2] /= l;

    // Difference between projected vector and observed bearing vector
    // We use the difference between unit vectors as an approximation
    // to the angle for small angles.
    residuals[0] = T(scale_) * (p[0] - T(bearing_vector_[0]));
    residuals[1] = T(scale_) * (p[1] - T(bearing_vector_[1]));
    residuals[2] = T(scale_) * (p[2] - T(bearing_vector_[2]));

    return true;
  }

  double *world_;
  double bearing_vector_[3];
  double scale_;
};

struct BasicRadialInternalParametersPriorError {
  BasicRadialInternalParametersPriorError(double focal_estimate,
                                          double focal_std_deviation,
                                          double k1_estimate,
                                          double k1_std_deviation,
                                          double k2_estimate,
                                          double k2_std_deviation)
      : log_focal_estimate_(log(focal_estimate))
      , focal_scale_(1.0 / focal_std_deviation)
      , k1_estimate_(k1_estimate)
      , k1_scale_(1.0 / k1_std_deviation)
      , k2_estimate_(k2_estimate)
      , k2_scale_(1.0 / k2_std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    residuals[0] = T(focal_scale_) * (log(parameters[BA_CAMERA_FOCAL]) - T(log_focal_estimate_));
    residuals[1] = T(k1_scale_) * (parameters[BA_CAMERA_K1] - T(k1_estimate_));
    residuals[2] = T(k2_scale_) * (parameters[BA_CAMERA_K2] - T(k2_estimate_));
    return true;
  }

  double log_focal_estimate_;
  double focal_scale_;
  double k1_estimate_;
  double k1_scale_;
  double k2_estimate_;
  double k2_scale_;
};

struct BrownInternalParametersPriorError {
  BrownInternalParametersPriorError(double focal_x_estimate,
                                    double focal_x_std_deviation,
                                    double focal_y_estimate,
                                    double focal_y_std_deviation,
                                    double c_x_estimate,
                                    double c_x_std_deviation,
                                    double c_y_estimate,
                                    double c_y_std_deviation,
                                    double k1_estimate,
                                    double k1_std_deviation,
                                    double k2_estimate,
                                    double k2_std_deviation,
                                    double p1_estimate,
                                    double p1_std_deviation,
                                    double p2_estimate,
                                    double p2_std_deviation,
                                    double k3_estimate,
                                    double k3_std_deviation)
      : log_focal_x_estimate_(log(focal_x_estimate))
      , focal_x_scale_(1.0 / focal_x_std_deviation)
      , log_focal_y_estimate_(log(focal_y_estimate))
      , focal_y_scale_(1.0 / focal_y_std_deviation)
      , c_x_estimate_(c_x_estimate)
      , c_x_scale_(1.0 / c_x_std_deviation)
      , c_y_estimate_(c_y_estimate)
      , c_y_scale_(1.0 / c_y_std_deviation)
      , k1_estimate_(k1_estimate)
      , k1_scale_(1.0 / k1_std_deviation)
      , k2_estimate_(k2_estimate)
      , k2_scale_(1.0 / k2_std_deviation)
      , p1_estimate_(p1_estimate)
      , p1_scale_(1.0 / p1_std_deviation)
      , p2_estimate_(p2_estimate)
      , p2_scale_(1.0 / p2_std_deviation)
      , k3_estimate_(k3_estimate)
      , k3_scale_(1.0 / k3_std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    residuals[0] = T(focal_x_scale_) * (log(parameters[BA_BROWN_CAMERA_FOCAL_X]) - T(log_focal_x_estimate_));
    residuals[1] = T(focal_y_scale_) * (log(parameters[BA_BROWN_CAMERA_FOCAL_Y]) - T(log_focal_y_estimate_));
    residuals[2] = T(c_x_scale_) * (parameters[BA_BROWN_CAMERA_C_X] - T(c_x_estimate_));
    residuals[3] = T(c_y_scale_) * (parameters[BA_BROWN_CAMERA_C_Y] - T(c_y_estimate_));
    residuals[4] = T(k1_scale_) * (parameters[BA_BROWN_CAMERA_K1] - T(k1_estimate_));
    residuals[5] = T(k2_scale_) * (parameters[BA_BROWN_CAMERA_K2] - T(k2_estimate_));
    residuals[6] = T(p1_scale_) * (parameters[BA_BROWN_CAMERA_P1] - T(p1_estimate_));
    residuals[7] = T(p2_scale_) * (parameters[BA_BROWN_CAMERA_P2] - T(p2_estimate_));
    residuals[8] = T(k3_scale_) * (parameters[BA_BROWN_CAMERA_K3] - T(k3_estimate_));
    return true;
  }

  double log_focal_x_estimate_;
  double focal_x_scale_;
  double log_focal_y_estimate_;
  double focal_y_scale_;
  double c_x_estimate_;
  double c_x_scale_;
  double c_y_estimate_;
  double c_y_scale_;
  double k1_estimate_;
  double k1_scale_;
  double k2_estimate_;
  double k2_scale_;
  double p1_estimate_;
  double p1_scale_;
  double p2_estimate_;
  double p2_scale_;
  double k3_estimate_;
  double k3_scale_;
};


struct RotationPriorError {
  RotationPriorError(double *R_prior, double std_deviation)
      : R_prior_(R_prior)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    // Get rotation and translation values.
    const T* const R = shot + BA_SHOT_RX;
    T Rpt[3] = { -T(R_prior_[0]),
                 -T(R_prior_[1]),
                 -T(R_prior_[2]) };

    // Compute rotation residual: log( R Rp^t )
    T qR[4], qRpt[4], qR_Rpt[4];
    ceres::AngleAxisToQuaternion(R, qR);
    ceres::AngleAxisToQuaternion(Rpt, qRpt);
    ceres::QuaternionProduct(qR, qRpt, qR_Rpt);
    T R_Rpt[3];
    ceres::QuaternionToAngleAxis(qR_Rpt, R_Rpt);

    residuals[0] = T(scale_) * R_Rpt[0];
    residuals[1] = T(scale_) * R_Rpt[1];
    residuals[2] = T(scale_) * R_Rpt[2];

    return true;
  }

  double *R_prior_;
  double scale_;
};

struct TranslationPriorError {
  TranslationPriorError(double *translation_prior, double std_deviation)
      : translation_prior_(translation_prior)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    residuals[0] = T(scale_) * (T(translation_prior_[0]) - shot[BA_SHOT_TX]);
    residuals[1] = T(scale_) * (T(translation_prior_[1]) - shot[BA_SHOT_TY]);
    residuals[2] = T(scale_) * (T(translation_prior_[2]) - shot[BA_SHOT_TZ]);
    return true;
  }

  double *translation_prior_;
  double scale_;
};

struct PositionPriorError {
  PositionPriorError(double *position_prior, double std_deviation)
      : position_prior_(position_prior)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    T Rt[3] = { -shot[BA_SHOT_RX], -shot[BA_SHOT_RY], -shot[BA_SHOT_RZ] };
    T p[3];
    ceres::AngleAxisRotatePoint(Rt, shot + BA_SHOT_TX, p);

    residuals[0] = T(scale_) * (p[0] + T(position_prior_[0]));
    residuals[1] = T(scale_) * (p[1] + T(position_prior_[1]));
    residuals[2] = T(scale_) * (p[2] + T(position_prior_[2]));
    return true;
  }

  double *position_prior_;
  double scale_;
};

struct UnitTranslationPriorError {
  UnitTranslationPriorError() {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    const T* const t = shot + 3;
    residuals[0] = log(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    return true;
  }
};

struct PointPositionPriorError {
  PointPositionPriorError(double *position, double std_deviation)
      : position_(position)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const p, T* residuals) const {
    residuals[0] = T(scale_) * (p[0] - T(position_[0]));
    residuals[1] = T(scale_) * (p[1] - T(position_[1]));
    residuals[2] = T(scale_) * (p[2] - T(position_[2]));
    return true;
  }

  double *position_;
  double scale_;
};

ceres::LossFunction *CreateLossFunction(std::string name, double threshold) {
  if (name.compare("TrivialLoss") == 0) {
    return new ceres::TrivialLoss();
  } else if (name.compare("HuberLoss") == 0) {
    return new ceres::HuberLoss(threshold);
  } else if (name.compare("SoftLOneLoss") == 0) {
    return new ceres::SoftLOneLoss(threshold);
  } else if (name.compare("CauchyLoss") == 0) {
    return new ceres::CauchyLoss(threshold);
  } else if (name.compare("ArctanLoss") == 0) {
    return new ceres::ArctanLoss(threshold);
  } else if (name.compare("TruncatedLoss") == 0) {
    return new TruncatedLoss(threshold);
  }
  return NULL;
}


ceres::LinearSolverType LinearSolverTypeFromNamae(std::string name) {
  if (name.compare("DENSE_QR") == 0) {
    return ceres::DENSE_QR;
  } else if (name.compare("DENSE_NORMAL_CHOLESKY") == 0) {
    return ceres::DENSE_NORMAL_CHOLESKY;
  } else if (name.compare("SPARSE_NORMAL_CHOLESKY") == 0) {
    return ceres::SPARSE_NORMAL_CHOLESKY;
  } else if (name.compare("CGNR") == 0) {
    return ceres::CGNR;
  } else if (name.compare("DENSE_SCHUR") == 0) {
    return ceres::DENSE_SCHUR;
  } else if (name.compare("SPARSE_SCHUR") == 0) {
    return ceres::SPARSE_SCHUR;
  } else if (name.compare("ITERATIVE_SCHUR") == 0) {
    return ceres::ITERATIVE_SCHUR;
  }
  return ceres::SPARSE_SCHUR;
}


// A bundle adjustment class for optimizing the problem
//
//    sum_p ( reprojection_error(p) / reprojection_error_sd )^2
//  + sum_c ( (focal - focal_prior) / focal_prior_sd )^2
//
class BundleAdjuster {
 public:
  BundleAdjuster() {
    unit_translation_shot_ = NULL;
    loss_function_ = "TrivialLoss";
    loss_function_threshold_ = 1;
    reprojection_error_sd_ = 1;
    focal_prior_sd_ = 1;
    c_prior_sd_ = 1;
    k1_sd_ = 1;
    k2_sd_ = 1;
    p1_sd_ = 1;
    p2_sd_ = 1;
    k3_sd_ = 1;
    compute_covariances_ = false;
    covariance_estimation_valid_ = false;
    compute_reprojection_errors_ = true;
    max_num_iterations_ = 50;
    num_threads_ = 1;
    linear_solver_type_ = "SPARSE_SCHUR";
  }

  // Disable copy constructor
  BundleAdjuster(const BundleAdjuster &) = delete;

  virtual ~BundleAdjuster() {}

  BAPerspectiveCamera GetPerspectiveCamera(const std::string &id) {
    return *(BAPerspectiveCamera *)cameras_[id].get();
  }

  BABrownPerspectiveCamera GetBrownPerspectiveCamera(const std::string &id) {
    return *(BABrownPerspectiveCamera *)cameras_[id].get();
  }

  BAFisheyeCamera GetFisheyeCamera(const std::string &id) {
    return *(BAFisheyeCamera *)cameras_[id].get();
  }

  BAEquirectangularCamera GetEquirectangularCamera(const std::string &id) {
    return *(BAEquirectangularCamera *)cameras_[id].get();
  }

  BAShot GetShot(const std::string &id) {
    return shots_[id];
  }

  BAPoint GetPoint(const std::string &id) {
    return points_[id];
  }

  void AddPerspectiveCamera(
      const std::string &id,
      double focal,
      double k1,
      double k2,
      double focal_prior,
      double k1_prior,
      double k2_prior,
      bool constant) {
    cameras_[id] = std::unique_ptr<BAPerspectiveCamera>(new BAPerspectiveCamera());
    BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*cameras_[id]);
    c.id = id;
    c.parameters[BA_CAMERA_FOCAL] = focal;
    c.parameters[BA_CAMERA_K1] = k1;
    c.parameters[BA_CAMERA_K2] = k2;
    c.constant = constant;
    c.focal_prior = focal_prior;
    c.k1_prior = k1_prior;
    c.k2_prior = k2_prior;
  }

  void AddBrownPerspectiveCamera(const BABrownPerspectiveCamera &c) {
    cameras_[c.id] = std::unique_ptr<BABrownPerspectiveCamera>(new BABrownPerspectiveCamera(c));
  }

  void AddFisheyeCamera(
      const std::string &id,
      double focal,
      double k1,
      double k2,
      double focal_prior,
      double k1_prior,
      double k2_prior,
      bool constant) {
    cameras_[id] = std::unique_ptr<BAFisheyeCamera>(new BAFisheyeCamera());
    BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*cameras_[id]);
    c.id = id;
    c.parameters[BA_CAMERA_FOCAL] = focal;
    c.parameters[BA_CAMERA_K1] = k1;
    c.parameters[BA_CAMERA_K2] = k2;
    c.constant = constant;
    c.focal_prior = focal_prior;
    c.k1_prior = k1_prior;
    c.k2_prior = k2_prior;
  }

  void AddEquirectangularCamera(
      const std::string &id) {
    cameras_[id] = std::unique_ptr<BAEquirectangularCamera>(new BAEquirectangularCamera());
    BAEquirectangularCamera &c = static_cast<BAEquirectangularCamera &>(*cameras_[id]);
    c.id = id;
  }

  void AddShot(
      const std::string &id,
      const std::string &camera,
      double rx,
      double ry,
      double rz,
      double tx,
      double ty,
      double tz,
      bool constant) {
    BAShot s;
    s.id = id;
    s.camera = camera;
    s.parameters[BA_SHOT_RX] = rx;
    s.parameters[BA_SHOT_RY] = ry;
    s.parameters[BA_SHOT_RZ] = rz;
    s.parameters[BA_SHOT_TX] = tx;
    s.parameters[BA_SHOT_TY] = ty;
    s.parameters[BA_SHOT_TZ] = tz;
    s.constant = constant;
    shots_[id] = s;
  }

  void AddPoint(
      const std::string &id,
      double x,
      double y,
      double z,
      bool constant) {
    BAPoint p;
    p.id = id;
    p.coordinates[0] = x;
    p.coordinates[1] = y;
    p.coordinates[2] = z;
    p.constant = constant;
    p.reprojection_error = -1;
    points_[id] = p;
  }

  void AddObservation(
      const std::string &shot,
      const std::string &point,
      double x,
      double y) {
    BAObservation o;
    o.shot = &shots_[shot];
    o.camera = cameras_[o.shot->camera].get();
    o.point = &points_[point];
    o.coordinates[0] = x;
    o.coordinates[1] = y;
    observations_.push_back(o);
  }

  void AddRotationPrior(
      const std::string &shot_id,
      double rx,
      double ry,
      double rz,
      double std_deviation) {
    BARotationPrior p;
    p.shot = &shots_[shot_id];
    p.rotation[0] = rx;
    p.rotation[1] = ry;
    p.rotation[2] = rz;
    p.std_deviation = std_deviation;
    rotation_priors_.push_back(p);
  }

  void AddTranslationPrior(
      const std::string &shot_id,
      double tx,
      double ty,
      double tz,
      double std_deviation) {
    BATranslationPrior p;
    p.shot = &shots_[shot_id];
    p.translation[0] = tx;
    p.translation[1] = ty;
    p.translation[2] = tz;
    p.std_deviation = std_deviation;
    translation_priors_.push_back(p);
  }

  void AddPositionPrior(
      const std::string &shot_id,
      double x,
      double y,
      double z,
      double std_deviation) {
    BAPositionPrior p;
    p.shot = &shots_[shot_id];
    p.position[0] = x;
    p.position[1] = y;
    p.position[2] = z;
    p.std_deviation = std_deviation;
    position_priors_.push_back(p);
  }

  void AddPointPositionPrior(
      const std::string &point_id,
      double x,
      double y,
      double z,
      double std_deviation) {
    BAPointPositionPrior p;
    p.point = &points_[point_id];
    p.position[0] = x;
    p.position[1] = y;
    p.position[2] = z;
    p.std_deviation = std_deviation;
    point_position_priors_.push_back(p);
  }

  void AddGroundControlPointObservation(
      const std::string &shot,
      double x3d,
      double y3d,
      double z3d,
      double x2d,
      double y2d) {
    BAGroundControlPointObservation o;
    o.shot = &shots_[shot];
    o.camera = cameras_[o.shot->camera].get();
    o.coordinates3d[0] = x3d;
    o.coordinates3d[1] = y3d;
    o.coordinates3d[2] = z3d;
    o.coordinates2d[0] = x2d;
    o.coordinates2d[1] = y2d;
    gcp_observations_.push_back(o);
  }

  void SetOriginShot(const std::string &shot_id) {
    BAShot *shot = &shots_[shot_id];
    for (int i = 0; i < 6; ++i) shot->parameters[0] = 0;
    shot->constant = true;
  }

  void SetUnitTranslationShot(const std::string &shot_id) {
    unit_translation_shot_ = &shots_[shot_id];
  }

  void SetLossFunction(const std::string &function_name,
                       double threshold) {
    loss_function_ = function_name;
    loss_function_threshold_ = threshold;
  }

  void SetReprojectionErrorSD(double sd) {
    reprojection_error_sd_ = sd;
  }

  void SetMaxNumIterations(int miter) {
    max_num_iterations_ = miter;
  }

  void SetNumThreads(int n) {
    num_threads_ = n;
  }

  void SetLinearSolverType(std::string t) {
    linear_solver_type_ = t;
  }

  void SetInternalParametersPriorSD(double focal_sd, double c_sd, double k1_sd, double k2_sd, double p1_sd, double p2_sd, double k3_sd) {
    focal_prior_sd_ = focal_sd;
    c_prior_sd_ = c_sd;
    k1_sd_ = k1_sd;
    k2_sd_ = k2_sd;
    p1_sd_ = p1_sd;
    p2_sd_ = p2_sd;
    k3_sd_ = k3_sd;
  }

  void SetComputeCovariances(bool v) {
    compute_covariances_ = v;
  }

  bool GetCovarianceEstimationValid() {
    return covariance_estimation_valid_;
  }

  void SetComputeReprojectionErrors(bool v) {
    compute_reprojection_errors_ = v;
  }

  void Run() {
    ceres::LossFunction *loss = CreateLossFunction(loss_function_, loss_function_threshold_);
    ceres::Problem problem;

    // Init parameter blocks.
    for (auto &i : cameras_) {
      if (i.second->constant) {
        switch (i.second->type()) {
          case BA_PERSPECTIVE_CAMERA:
          {
            BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*i.second);
            problem.AddParameterBlock(c.parameters, BA_CAMERA_NUM_PARAMS);
            problem.SetParameterBlockConstant(c.parameters);
            break;
          }
          case BA_BROWN_PERSPECTIVE_CAMERA:
          {
            BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*i.second);
            problem.AddParameterBlock(c.parameters, BA_BROWN_CAMERA_NUM_PARAMS);
            problem.SetParameterBlockConstant(c.parameters);
            break;
          }
          case BA_FISHEYE_CAMERA:
          {
            BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*i.second);
            problem.AddParameterBlock(c.parameters, BA_CAMERA_NUM_PARAMS);
            problem.SetParameterBlockConstant(c.parameters);
            break;
          }
          case BA_EQUIRECTANGULAR_CAMERA:
            // No parameters for now
            break;
        }
      }
    }
    for (auto &i : shots_) {
      if (i.second.constant) {
        problem.AddParameterBlock(i.second.parameters, BA_SHOT_NUM_PARAMS);
        problem.SetParameterBlockConstant(i.second.parameters);
      }
    }
    for (auto &i : points_) {
      if (i.second.constant) {
        problem.AddParameterBlock(i.second.coordinates, 3);
        problem.SetParameterBlockConstant(i.second.coordinates);
      }
    }

    // Add reprojection error blocks
    for (auto &observation : observations_) {
      switch (observation.camera->type()) {
        case BA_PERSPECTIVE_CAMERA:
        {
          BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*observation.camera);
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<PerspectiveReprojectionError, 2, 3, 6, 3>(
                  new PerspectiveReprojectionError(observation.coordinates[0],
                                                   observation.coordinates[1],
                                                   reprojection_error_sd_));

          problem.AddResidualBlock(cost_function,
                                   loss,
                                   c.parameters,
                                   observation.shot->parameters,
                                   observation.point->coordinates);
          break;
        }
        case BA_BROWN_PERSPECTIVE_CAMERA:
        {
          BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*observation.camera);
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<BrownPerspectiveReprojectionError, 2, 9, 6, 3>(
                  new BrownPerspectiveReprojectionError(observation.coordinates[0],
                                                        observation.coordinates[1],
                                                        reprojection_error_sd_));

          problem.AddResidualBlock(cost_function,
                                   loss,
                                   c.parameters,
                                   observation.shot->parameters,
                                   observation.point->coordinates);
          break;
        }
        case BA_FISHEYE_CAMERA:
        {
          BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*observation.camera);
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<FisheyeReprojectionError, 2, 3, 6, 3>(
                  new FisheyeReprojectionError(observation.coordinates[0],
                                               observation.coordinates[1],
                                               reprojection_error_sd_));

          problem.AddResidualBlock(cost_function,
                                   loss,
                                   c.parameters,
                                   observation.shot->parameters,
                                   observation.point->coordinates);
          break;
        }
        case BA_EQUIRECTANGULAR_CAMERA:
        {
          BAEquirectangularCamera &c = static_cast<BAEquirectangularCamera &>(*observation.camera);
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<EquirectangularReprojectionError, 3, 6, 3>(
                  new EquirectangularReprojectionError(observation.coordinates[0],
                                                       observation.coordinates[1],
                                                       reprojection_error_sd_));

          problem.AddResidualBlock(cost_function,
                                   loss,
                                   observation.shot->parameters,
                                   observation.point->coordinates);
          break;
        }
      }
    }

    // Add rotation priors
    for (auto &rp : rotation_priors_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<RotationPriorError, 3, 6>(
              new RotationPriorError(rp.rotation, rp.std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               rp.shot->parameters);
    }

    // Add translation priors
    for (auto &tp : translation_priors_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<TranslationPriorError, 3, 6>(
              new TranslationPriorError(tp.translation, tp.std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               tp.shot->parameters);
    }

    // Add position priors
    for (auto &pp : position_priors_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<PositionPriorError, 3, 6>(
              new PositionPriorError(pp.position, pp.std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               pp.shot->parameters);
    }

    // Add point position priors
    for (auto &pp : point_position_priors_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<PointPositionPriorError, 3, 3>(
              new PointPositionPriorError(pp.position, pp.std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               pp.point->coordinates);
    }

    // Add ground control point observations
    for (auto &observation : gcp_observations_) {
      switch (observation.camera->type()) {
        case BA_PERSPECTIVE_CAMERA:
        {
          BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*observation.camera);
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<GCPPerspectiveProjectionError, 2, 3, 6>(
                  new GCPPerspectiveProjectionError(observation.coordinates3d,
                                                    observation.coordinates2d,
                                                    reprojection_error_sd_));
          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   c.parameters,
                                   observation.shot->parameters);
          break;
        }
        case BA_BROWN_PERSPECTIVE_CAMERA:
        {
          BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*observation.camera);
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<GCPBrownPerspectiveProjectionError, 2, 9, 6>(
                  new GCPBrownPerspectiveProjectionError(observation.coordinates3d,
                                                         observation.coordinates2d,
                                                         reprojection_error_sd_));
          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   c.parameters,
                                   observation.shot->parameters);
          break;
        }
        case BA_FISHEYE_CAMERA:
        {
          std::cerr << "NotImplemented: GCP for fisheye cameras\n";
          break;
        }
        case BA_EQUIRECTANGULAR_CAMERA:
        {
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<GCPEquirectangularProjectionError, 3, 6>(
                  new GCPEquirectangularProjectionError(observation.coordinates3d,
                                                        observation.coordinates2d,
                                                        reprojection_error_sd_));

          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   observation.shot->parameters);
          break;
        }
      }
    }

    // Add internal parameter priors blocks
    for (auto &i : cameras_) {
      switch (i.second->type()) {
        case BA_PERSPECTIVE_CAMERA:
        {
          BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*i.second);

          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<BasicRadialInternalParametersPriorError, 3, 3>(
                  new BasicRadialInternalParametersPriorError(c.focal_prior, focal_prior_sd_,
                                                              c.k1_prior, k1_sd_,
                                                              c.k2_prior, k2_sd_));

          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   c.parameters);
          break;
        }
        case BA_BROWN_PERSPECTIVE_CAMERA:
        {
          BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*i.second);

          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<BrownInternalParametersPriorError, 9, 9>(
                  new BrownInternalParametersPriorError(c.focal_x_prior, focal_prior_sd_,
                                                        c.focal_y_prior, focal_prior_sd_,
                                                        c.c_x_prior, c_prior_sd_,
                                                        c.c_y_prior, c_prior_sd_,
                                                        c.k1_prior, k1_sd_,
                                                        c.k2_prior, k2_sd_,
                                                        c.p1_prior, p1_sd_,
                                                        c.p2_prior, p2_sd_,
                                                        c.k3_prior, k3_sd_));

          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   c.parameters);
          break;
        }
        case BA_FISHEYE_CAMERA:
        {
          BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*i.second);

          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<BasicRadialInternalParametersPriorError, 3, 3>(
                  new BasicRadialInternalParametersPriorError(c.focal_prior, focal_prior_sd_,
                                                              c.k1_prior, k1_sd_,
                                                              c.k2_prior, k2_sd_));

          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   c.parameters);
          break;
        }
        case BA_EQUIRECTANGULAR_CAMERA:
          break;
      }
    }

    // Add unit translation block
    if (unit_translation_shot_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<UnitTranslationPriorError, 1, 6>(
              new UnitTranslationPriorError());

      problem.AddResidualBlock(cost_function,
                               NULL,
                               unit_translation_shot_->parameters);
    }

    // Solve
    ceres::Solver::Options options;
    options.linear_solver_type = LinearSolverTypeFromNamae(linear_solver_type_);
    options.num_threads = num_threads_;
    options.num_linear_solver_threads = num_threads_;
    options.max_num_iterations = max_num_iterations_;

    ceres::Solve(options, &problem, &last_run_summary_);

    if (compute_covariances_) {
      ComputeCovariances(&problem);
    }
    if (compute_reprojection_errors_) {
      ComputeReprojectionErrors();
    }
  }

  void ComputeCovariances(ceres::Problem *problem) {
    ceres::Covariance::Options options;
    ceres::Covariance covariance(options);

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    for (auto &i : shots_) {
      covariance_blocks.push_back(std::make_pair(i.second.parameters, i.second.parameters));
    }

    bool worked = covariance.Compute(covariance_blocks, problem);

    if (worked) {
      for (auto &i : shots_) {
        covariance_estimation_valid_ = true;
        covariance.GetCovarianceBlock(i.second.parameters, i.second.parameters, i.second.covariance);
      }
    } else { // If covariance estimation failed, use a default value
      for (auto &i : shots_) {
        covariance_estimation_valid_ = false;
        for (int k = 0; k < 6 * 6; ++k) {
          i.second.covariance[k] = 0.0;
        }
        double default_rotation_variance = 1e-5;
        double default_translation_variance = 1e-2;
        i.second.covariance[6 * 0 + 0] = default_rotation_variance;
        i.second.covariance[6 * 1 + 1] = default_rotation_variance;
        i.second.covariance[6 * 2 + 2] = default_rotation_variance;
        i.second.covariance[6 * 3 + 3] = default_translation_variance;
        i.second.covariance[6 * 4 + 4] = default_translation_variance;
        i.second.covariance[6 * 5 + 5] = default_translation_variance;
      }
    }
  }

  void ComputeReprojectionErrors() {
    // Init errors
    for (auto &i : points_) {
      i.second.reprojection_error = 0;
    }

    // Sum over all observations
    for (int i = 0; i < observations_.size(); ++i) {
      switch (observations_[i].camera->type()) {
        case BA_PERSPECTIVE_CAMERA:
        {
          BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*observations_[i].camera);

          PerspectiveReprojectionError pre(observations_[i].coordinates[0],
                                           observations_[i].coordinates[1],
                                           1.0);
          double residuals[2];
          pre(c.parameters,
              observations_[i].shot->parameters,
              observations_[i].point->coordinates,
              residuals);
          double error = sqrt(residuals[0] * residuals[0] + residuals[1] * residuals[1]);
          observations_[i].point->reprojection_error =
              std::max(observations_[i].point->reprojection_error, error);
          break;
        }
        case BA_BROWN_PERSPECTIVE_CAMERA:
        {
          BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*observations_[i].camera);

          BrownPerspectiveReprojectionError bpre(observations_[i].coordinates[0],
                                                 observations_[i].coordinates[1],
                                                 1.0);
          double residuals[2];
          bpre(c.parameters,
               observations_[i].shot->parameters,
               observations_[i].point->coordinates,
               residuals);
          double error = sqrt(residuals[0] * residuals[0] + residuals[1] * residuals[1]);
          observations_[i].point->reprojection_error =
              std::max(observations_[i].point->reprojection_error, error);
          break;
        }
        case BA_FISHEYE_CAMERA:
        {
          BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*observations_[i].camera);

          FisheyeReprojectionError pre(observations_[i].coordinates[0],
                                       observations_[i].coordinates[1],
                                       1.0);
          double residuals[2];
          pre(c.parameters,
              observations_[i].shot->parameters,
              observations_[i].point->coordinates,
              residuals);
          double error = sqrt(residuals[0] * residuals[0] + residuals[1] * residuals[1]);
          observations_[i].point->reprojection_error =
              std::max(observations_[i].point->reprojection_error, error);
          break;
        }
        case BA_EQUIRECTANGULAR_CAMERA:
        {
          BAEquirectangularCamera &c = static_cast<BAEquirectangularCamera &>(*observations_[i].camera);

          EquirectangularReprojectionError ere(observations_[i].coordinates[0],
                                               observations_[i].coordinates[1],
                                               1.0);
          double residuals[3];
          ere(observations_[i].shot->parameters,
              observations_[i].point->coordinates,
              residuals);
          double error = sqrt(residuals[0] * residuals[0] + residuals[1] * residuals[1] + residuals[2] * residuals[2]);
          observations_[i].point->reprojection_error =
              std::max(observations_[i].point->reprojection_error, error);
          break;
        }
      }
    }
  }

  std::string BriefReport() {
    return last_run_summary_.BriefReport();
  }
  std::string FullReport() {
    return last_run_summary_.FullReport();
  }

 private:
  std::map<std::string, std::unique_ptr<BACamera> > cameras_;
  std::map<std::string, BAShot> shots_;
  std::map<std::string, BAPoint> points_;

  std::vector<BAObservation> observations_;
  std::vector<BARotationPrior> rotation_priors_;
  std::vector<BATranslationPrior> translation_priors_;
  std::vector<BAPositionPrior> position_priors_;
  std::vector<BAPointPositionPrior> point_position_priors_;
  std::vector<BAGroundControlPointObservation> gcp_observations_;

  BAShot *unit_translation_shot_;

  std::string loss_function_;
  double loss_function_threshold_;
  double reprojection_error_sd_;
  double focal_prior_sd_;
  double c_prior_sd_;
  double k1_sd_;
  double k2_sd_;
  double p1_sd_;
  double p2_sd_;
  double k3_sd_;
  bool compute_covariances_;
  bool covariance_estimation_valid_;
  bool compute_reprojection_errors_;
  int max_num_iterations_;
  int num_threads_;
  std::string linear_solver_type_;

  ceres::Solver::Summary last_run_summary_;
};
