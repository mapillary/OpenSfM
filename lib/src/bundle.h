#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <json/json.h>

extern "C" {
#include <string.h>
}

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"



// Return a newly allocated char array with the file contents.
static bool FileContents(const char *filename, std::string *file_contents) {
  std::ifstream file_stream;
  file_stream.open(filename, std::ios::binary );

  if (!file_stream.good()) return false;

  // get length of file:
  file_stream.seekg(0, std::ios::end);
  int file_length = file_stream.tellg();
  file_stream.seekg(0, std::ios::beg);

  // read data as a block:
  file_contents->resize(file_length);
  file_stream.read(&(*file_contents)[0], file_length);
  file_stream.close();

  return true;
}

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


// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed to be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y, double focal=-1)
      : observed_x_(observed_x)
      , observed_y_(observed_y)
      , focal_(focal)
  {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const shot,
                  const T* const point,
                  T* residuals) const {
    // shot[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(shot, point, p);

    // shot[3,4,5] are the translation.
    p[0] += shot[3];
    p[1] += shot[4];
    p[2] += shot[5];

    // Project.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[1];
    const T& l2 = camera[2];
    T r2 = xp * xp + yp * yp;
    T distortion = T(1.0) ;// + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    const T& focal = (focal_ > 0.0) ? T(focal_) : camera[0];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x_);
    residuals[1] = predicted_y - T(observed_y_);

    return true;
  }

  double observed_x_;
  double observed_y_;
  double focal_;
};


struct FocalPriorError {
  FocalPriorError(double estimate, double std_deviation)
      : estimate_(estimate)
      , scale_(1.0 / std_deviation / sqrt(2))
  {}

  template <typename T>
  bool operator()(const T* const focal, T* residuals) const {
    residuals[0] = T(scale_) * (*focal - T(estimate_));
    return true;
  }

  double estimate_;
  double scale_;
};


struct GPSPriorError {
  GPSPriorError(double x, double y, double z, double std_deviation)
      : x_(x), y_(y), z_(z)
      , scale_(1.0 / std_deviation / sqrt(2))
  {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    // shot[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(shot, shot + 3, p);

    residuals[0] = T(scale_) * (-p[0] - T(x_));
    residuals[1] = T(scale_) * (-p[1] - T(y_));
    residuals[2] = T(scale_) * (-p[2] - T(z_));
    return true;
  }

  double x_, y_, z_;
  double scale_;
};

enum {
  BA_CAMERA_FOCAL,
  BA_CAMERA_K1,
  BA_CAMERA_K2,
  BA_CAMERA_NUM_PARAMS
};

struct BACamera {
  double parameters[BA_CAMERA_NUM_PARAMS];
  double width, height;
  double exif_focal;
  std::string id;

  double GetFocal() { return parameters[BA_CAMERA_FOCAL]; }
  double GetK1() { return parameters[BA_CAMERA_K1]; }
  double GetK2() { return parameters[BA_CAMERA_K2]; }
  void SetFocal(double v) { parameters[BA_CAMERA_FOCAL] = v; }
  void SetK1(double v) { parameters[BA_CAMERA_K1] = v; }
  void SetK2(double v) { parameters[BA_CAMERA_K2] = v; }
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
  double gps_x, gps_y, gps_z;
  double gps_dop;
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
};

struct BAPoint {
  double coordinates[3];
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

// Read and write the BA problem from a json file.
class BundleAdjuster {
 public:
  ~BundleAdjuster() {}

  BACamera GetCamera(const std::string &id) {
    return cameras_[id];
  }

  BAShot GetShot(const std::string &id) {
    return shots_[id];
  }

  BAPoint GetPoint(const std::string &id) {
    return points_[id];
  }

  void AddCamera(
      const std::string &id,
      double focal,
      double k1,
      double k2,
      double width,
      double height,
      double exif_focal) {
    BACamera c;   
    c.id = id;
    c.parameters[BA_CAMERA_FOCAL] = focal;
    c.parameters[BA_CAMERA_K1] = k1;
    c.parameters[BA_CAMERA_K2] = k2;
    c.height = height;
    c.width = width;
    c.exif_focal = exif_focal;
    cameras_[id] = c;
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
      double gpsx,
      double gpsy,
      double gpsz,
      double gps_dop) {
    BAShot s;
    s.id = id;
    s.camera = camera;
    s.parameters[BA_SHOT_RX] = rx;
    s.parameters[BA_SHOT_RY] = ry;
    s.parameters[BA_SHOT_RZ] = rz;
    s.parameters[BA_SHOT_TX] = tx;
    s.parameters[BA_SHOT_TY] = ty;
    s.parameters[BA_SHOT_TZ] = tz;
    s.gps_x = gpsx;
    s.gps_y = gpsy;
    s.gps_z = gpsz;
    s.gps_dop = gps_dop;
    shots_[id] = s;
  }

  void AddPoint(
      const std::string &id,
      double x,
      double y,
      double z) {
    BAPoint p;
    p.id = id;
    p.coordinates[0] = x;
    p.coordinates[1] = y;
    p.coordinates[2] = z;
    points_[id] = p;
  }

  void AddObservation(
      const std::string &shot,
      const std::string &point,
      double x,
      double y) {
    BAObservation o;
    o.shot = &shots_[shot];
    o.camera = &cameras_[o.shot->camera];
    o.point = &points_[point];
    o.coordinates[0] = x;
    o.coordinates[1] = y;
    observations_.push_back(o);
  }

  void SetLossFunction(const std::string &function_name,
                       double threshold) {
    loss_function_ = function_name;
    loss_function_threshold_ = threshold;
  }

  void SetFocalPriorSD(double sd) {
    focal_prior_sd_ = sd;
  }


  bool LoadJson(const char *tracks, const char *reconstruction) {

    std::string file_contents;
    FileContents(reconstruction, &file_contents);

    Json::Value root;   // will contains the root value after parsing.
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(file_contents, root);
    if (!parsingSuccessful) {
      // report to the user the failure and their locations in the document.
      std::cout  << "Failed to parse json\n"
                 << reader.getFormatedErrorMessages();
      return false;
    }


    //////////////////////////////////////////////////////////////////
    // Read cameras
    //////////////////////////////////////////////////////////////////
    Json::Value cameras = root["cameras"];
    for(Json::ValueIterator i = cameras.begin(); i != cameras.end(); ++i) {
      AddCamera(
          i.key().asString(),
          (*i)["focal"].asDouble(),
          (*i)["k1"].asDouble(),
          (*i)["k2"].asDouble(),
          (*i)["height"].asDouble(),
          (*i)["width"].asDouble(),
          (*i)["exif_focal"].asDouble()
      );
    }


    //////////////////////////////////////////////////////////////////
    // Read shots
    //////////////////////////////////////////////////////////////////
    Json::Value shots = root["shots"];
    for(Json::ValueIterator i = shots.begin(); i != shots.end(); ++i) {
      AddShot(
          i.key().asString(),
          (*i)["camera"].asString(),
          (*i)["rotation"][0].asDouble(),
          (*i)["rotation"][1].asDouble(),
          (*i)["rotation"][2].asDouble(),
          (*i)["translation"][0].asDouble(),
          (*i)["translation"][1].asDouble(),
          (*i)["translation"][2].asDouble(),
          (*i)["gps_position"][0].asDouble(),
          (*i)["gps_position"][1].asDouble(),
          (*i)["gps_position"][2].asDouble(),
          (*i)["gps_dop"].asDouble()
      );
      shots_[i.key().asString()].exif_orientation = (*i)["exif_orientation"].asInt();
    }


    //////////////////////////////////////////////////////////////////
    // Read points
    //////////////////////////////////////////////////////////////////
    Json::Value points = root["points"];
    for(Json::ValueIterator i = points.begin(); i != points.end(); ++i) {
      AddPoint(
          i.key().asString(),
          (*i)["coordinates"][0].asDouble(),
          (*i)["coordinates"][1].asDouble(),
          (*i)["coordinates"][2].asDouble()
      );
    }


    //////////////////////////////////////////////////////////////////
    // Read observations
    //////////////////////////////////////////////////////////////////
    FILE* fptr = fopen(tracks, "r");
    if (fptr == NULL) {
      return false;
    };

    while (true) {
      char shot_id[1000];
      char point_id[1000];
      int oid;
      double x, y;
      int n = fscanf(fptr, "%[^\t]\t%[^\t]\t%d\t%lg\t%lg\n", shot_id, point_id, &oid, &x, &y);

      if (n != 5) break;

      if (shots_.count(shot_id) && points_.count(point_id)) {
        AddObservation(shot_id, point_id, x, y);
      }
    }

    std::cout << "Bundle starts " << cameras_.size() << " cameras, "
              << shots_.size() << " shots, "
              << points_.size() << " points, "
              << observations_.size() << " observations" << std::endl;

    return true;
  }

  bool SaveJson(const char *dest) {
    Json::Value root;

    // Cameras.
    Json::Value cameras;
    for (auto &i : cameras_) {
      Json::Value camera;
      camera["exif_focal"] = i.second.exif_focal;
      camera["width"] = i.second.width;
      camera["height"] = i.second.height;
      camera["focal"] = i.second.parameters[0];
      camera["k1"] = i.second.parameters[1];
      camera["k2"] = i.second.parameters[2];
      cameras[i.second.id] = camera;
    }
    root["cameras"] = cameras;

    // Shots.
    Json::Value shots;
    for (auto &i : shots_) {
      Json::Value shot;

      shot["camera"] = i.second.camera;
      Json::Value Rarray(Json::arrayValue);
      Json::Value tarray(Json::arrayValue);
      Json::Value gpstarray(Json::arrayValue);
      for (int j = 0; j < 3; ++j)
        Rarray.append(i.second.parameters[j]);
      for (int j = 0; j < 3; ++j)
        tarray.append(i.second.parameters[3 + j]);
      gpstarray.append(i.second.gps_x);
      gpstarray.append(i.second.gps_y);
      gpstarray.append(i.second.gps_z);
      shot["rotation"] = Rarray;
      shot["translation"] = tarray;
      shot["gps_position"] = gpstarray;
      shot["gps_dop"] = i.second.gps_dop;
      shot["exif_orientation"] = i.second.exif_orientation;
      shots[i.second.id] = shot;
    }
    root["shots"] = shots;

    // Points.
    Json::Value points;
    for (auto &i : points_) {
      Json::Value point;
      Json::Value coordinates(Json::arrayValue);
      for (int j = 0; j < 3; ++j)
        coordinates.append(i.second.coordinates[j]);
      point["coordinates"] = coordinates;
      points[i.second.id] = point;
    }
    root["points"] = points;


    Json::StyledWriter writer;
    std::string json = writer.write(root);

    std::ofstream fout(dest);
    fout << json;

    return true;
  }


  void Run() {
    ceres::LossFunction *loss;
    if (loss_function_.compare("TruncatedLoss") == 0) {
      loss = new TruncatedLoss(loss_function_threshold_);
    } else if (loss_function_.compare("TrivialLoss") == 0) {
      loss = new ceres::TrivialLoss();
    } else if (loss_function_.compare("HuberLoss") == 0) {
      loss = new ceres::HuberLoss(loss_function_threshold_);
    } else if (loss_function_.compare("SoftLOneLoss") == 0) {
      loss = new ceres::SoftLOneLoss(loss_function_threshold_);
    } else if (loss_function_.compare("CauchyLoss") == 0) {
      loss = new ceres::CauchyLoss(loss_function_threshold_);
    } else if (loss_function_.compare("ArctanLoss") == 0) {
      loss = new ceres::ArctanLoss(loss_function_threshold_);
    }

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;

    for (int i = 0; i < observations_.size(); ++i) {
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.

      ceres::CostFunction* cost_function = 
          new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 6, 3>(
              new SnavelyReprojectionError(observations_[i].coordinates[0],
                                           observations_[i].coordinates[1]));

      problem.AddResidualBlock(cost_function,
                               loss,
                               observations_[i].camera->parameters,
                               observations_[i].shot->parameters,
                               observations_[i].point->coordinates);
    }

    for (auto &i : cameras_) {
      double exif_focal_sd_in_pixels = focal_prior_sd_ * i.second.width;
      ceres::CostFunction* cost_function = 
          new ceres::AutoDiffCostFunction<FocalPriorError, 1, 3>(
              new FocalPriorError(i.second.exif_focal, exif_focal_sd_in_pixels));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               i.second.parameters);
    }

    // for (auto &i : shots_) {
    //   ceres::CostFunction* cost_function = 
    //       new ceres::AutoDiffCostFunction<GPSPriorError, 3, 6>(
    //           new GPSPriorError(i.second.gps_position[0],
    //                             i.second.gps_position[1],
    //                             i.second.gps_position[2],
    //                             i.second.gps_dop));

    //   problem.AddResidualBlock(cost_function,
    //                            NULL,
    //                            i.second.parameters);
    // }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
  }

 private:
  std::map<std::string, BACamera> cameras_;
  std::map<std::string, BAShot> shots_;
  std::map<std::string, BAPoint> points_;
  std::vector<BAObservation> observations_;

  std::string loss_function_;
  double loss_function_threshold_;
  double focal_prior_sd_;
};


