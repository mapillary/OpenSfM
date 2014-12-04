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




struct Camera {
  double parameters[3];
  double width, height;
  double exif_focal;
  std::string id;
};

struct Shot {
  double parameters[6];
  double gps_position[3];
  double gps_dop;
  std::string camera;
  std::string id;
};

struct Point {
  double parameters[3];
  std::string id;
};

struct Observation {
  double coordinates[2];
  Camera *camera;
  Shot *shot;
  Point *point;
};

// Read and write the BA problem from a json file.
class BALProblem {
 public:
  ~BALProblem() {}

  int num_observations() const { return observations_.size(); }
  Observation* observations() { return &observations_[0]; }
  int num_cameras() const { return cameras_.size(); }
  Camera* cameras() { return &cameras_[0]; }
  int num_shots() const { return shots_.size(); }
  Shot* shots() { return &shots_[0]; }


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
    for(Json::ValueIterator i = cameras.begin() ; i != cameras.end() ; ++i) {
      Camera c;
      c.id = i.key().asString();
      c.parameters[0] = (*i)["focal"].asDouble();
      c.parameters[1] = (*i)["k1"].asDouble();
      c.parameters[2] = (*i)["k2"].asDouble();
      c.height = (*i)["height"].asDouble();
      c.width = (*i)["width"].asDouble();
      c.exif_focal = (*i)["exif_focal"].asDouble();
      cameras_.push_back(c);
    }
    for (int i = 0; i < cameras_.size(); ++i) {
      camera_by_id_[cameras_[i].id] = &cameras_[i];
    }
    std::cout << cameras_.size() << " cameras read." << std::endl;


    //////////////////////////////////////////////////////////////////
    // Read shots
    //////////////////////////////////////////////////////////////////
    Json::Value shots = root["shots"];
    for(Json::ValueIterator i = shots.begin() ; i != shots.end() ; ++i) {
      Shot s;
      s.id = i.key().asString();
      for (int j = 0; j < 3; ++j)
        s.parameters[j] = (*i)["rotation"][j].asDouble();
      for (int j = 0; j < 3; ++j)
        s.parameters[3 + j] = (*i)["translation"][j].asDouble();
      for (int j = 0; j < 3; ++j)
        s.gps_position[j] = (*i)["gps_position"][j].asDouble();
      s.gps_dop = (*i)["gps_dop"].asDouble();
      s.camera = (*i)["camera"].asString();
      shots_.push_back(s);
    }
    for (int i = 0; i < shots_.size(); ++i) {
      shot_by_id_[shots_[i].id] = &shots_[i];
    }
    std::cout << shots_.size() << " shots read." << std::endl;


    //////////////////////////////////////////////////////////////////
    // Read points
    //////////////////////////////////////////////////////////////////
    Json::Value points = root["points"];
    for(Json::ValueIterator i = points.begin() ; i != points.end() ; ++i) {
      Point p;
      p.id = i.key().asString();
      for (int j = 0; j < 3; ++j)
        p.parameters[j] = (*i)["coordinates"][j].asDouble();
      points_.push_back(p);
    }
    for (int i = 0; i < points_.size(); ++i) {
      point_by_id_[points_[i].id] = &points_[i];
    }
    std::cout << points_.size() << " points read." << std::endl;


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

      if (shot_by_id_.count(shot_id) && point_by_id_.count(point_id)) { 
        Observation o;
        o.shot = shot_by_id_[shot_id];
        o.camera = camera_by_id_[o.shot->camera];
        o.point = point_by_id_[point_id];
        o.coordinates[0] = x - o.camera->width / 2;
        o.coordinates[1] = y - o.camera->height / 2;
        observations_.push_back(o);
      }
    }
    std::cout << observations_.size() << " observations read." << std::endl;

    return true;
  }

  bool SaveJson(const char *dest) {
    Json::Value root;

    // Cameras.
    Json::Value cameras;
    for (int i = 0; i < cameras_.size(); ++i) {
      Json::Value camera;
      camera["exif_focal"] = cameras_[i].exif_focal;
      camera["width"] = cameras_[i].width;
      camera["height"] = cameras_[i].height;
      camera["focal"] = cameras_[i].parameters[0];
      camera["k1"] = cameras_[i].parameters[1];
      camera["k2"] = cameras_[i].parameters[2];
      cameras[cameras_[i].id] = camera;
    }
    root["cameras"] = cameras;

    // Shots.
    Json::Value shots;
    for (int i = 0; i < shots_.size(); ++i) {
      Json::Value shot;

      shot["camera"] = shots_[i].camera;
      Json::Value Rarray(Json::arrayValue);
      Json::Value tarray(Json::arrayValue);
      Json::Value gpstarray(Json::arrayValue);
      for (int j = 0; j < 3; ++j)
        Rarray.append(shots_[i].parameters[j]);
      for (int j = 0; j < 3; ++j)
        tarray.append(shots_[i].parameters[3 + j]);
      for (int j = 0; j < 3; ++j)
        gpstarray.append(shots_[i].gps_position[j]);
      shot["rotation"] = Rarray;
      shot["translation"] = tarray;
      shot["gps_position"] = gpstarray;
      shot["gps_dop"] = shots_[i].gps_dop;
      shots[shots_[i].id] = shot;
    }
    root["shots"] = shots;

    // Points.
    Json::Value points;
    for (int i = 0; i < points_.size(); ++i) {
      Json::Value point;
      Json::Value coordinates(Json::arrayValue);
      for (int j = 0; j < 3; ++j)
        coordinates.append(points_[i].parameters[j]);
      point["coordinates"] = coordinates;
      points[points_[i].id] = point;
    }
    root["points"] = points;


    Json::StyledWriter writer;
    std::string json = writer.write(root);

    std::ofstream fout(dest);
    fout << json;

    return true;
  }


 private:
  std::vector<Camera> cameras_;
  std::vector<Shot> shots_;
  std::vector<Point> points_;
  std::vector<Observation> observations_;

  std::map<std::string, Camera *> camera_by_id_;
  std::map<std::string, Shot *> shot_by_id_;
  std::map<std::string, Point *> point_by_id_;
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
