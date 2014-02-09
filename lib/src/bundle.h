// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2010, 2011, 2012 Google Inc. All rights reserved.
// http://code.google.com/p/ceres-solver/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>

extern "C" { 
#include <string.h>
#include <jansson.h>
}

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"



struct Camera {
  double parameters[3];
  double width, height;
  std::string id;
};

struct Shot {
  double parameters[6];
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

// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
 public:
  ~BALProblem() {}

  int num_observations() const { return observations_.size(); }
  const Observation* observations() const { return &observations_[0]; }


  bool LoadJson(const char *tracks, const char *reconstruction) {
    json_error_t error;    
    json_t *root = json_load_file(reconstruction, 0, &error);

    if(!root) {
        fprintf(stderr, "error: on line %d: %s\n", error.line, error.text);
        return false;
    }

    if(!json_is_object(root)) {
        fprintf(stderr, "error: root is not an object\n");
        json_decref(root);
        return false;
    }

    //////////////////////////////////////////////////////////////////
    // Read cameras
    //////////////////////////////////////////////////////////////////
    json_t *cameras = json_object_get(root, "cameras");
    const char *camera_id;
    json_t *camera;
    json_object_foreach(cameras, camera_id, camera) {
      Camera c;
      c.id = camera_id;
      c.parameters[0] = json_number_value(json_object_get(camera, "focal"));
      c.parameters[1] = json_number_value(json_object_get(camera, "k1"));
      c.parameters[2] = json_number_value(json_object_get(camera, "k2"));
      c.height = json_number_value(json_object_get(camera, "height"));
      c.width = json_number_value(json_object_get(camera, "width"));
      cameras_.push_back(c);
    }
    for (int i = 0; i < cameras_.size(); ++i) {
      camera_by_id_[cameras_[i].id] = &cameras_[i];
    }

    //////////////////////////////////////////////////////////////////
    // Read shots
    //////////////////////////////////////////////////////////////////
    json_t *shots = json_object_get(root, "shots");
    const char *shot_id;
    json_t *shot;
    json_object_foreach(shots, shot_id, shot) {
      Shot s;
      s.id = shot_id;
      json_t *Rarray = json_object_get(shot, "rotation");
      for (int j = 0; j < 3; ++j)
        s.parameters[j] = json_number_value(json_array_get(Rarray, j));
      json_t *tarray = json_object_get(shot, "translation");
      for (int j = 0; j < 3; ++j)
        s.parameters[3 + j] = json_number_value(json_array_get(tarray, j));
      s.camera = json_string_value(json_object_get(shot, "camera"));
      shots_.push_back(s);
    }
    for (int i = 0; i < shots_.size(); ++i) {
      shot_by_id_[shots_[i].id] = &shots_[i];
    }


    //////////////////////////////////////////////////////////////////
    // Read points
    //////////////////////////////////////////////////////////////////
    json_t *points = json_object_get(root, "points");
    const char *point_id;
    json_t *point;
    json_object_foreach(points, point_id, point) {
      Point p;
      p.id = point_id;
      json_t *coordinates = json_object_get(point, "coordinates");
      for (int j = 0; j < 3; ++j)
        p.parameters[j] = json_number_value(json_array_get(coordinates, j));
      points_.push_back(p);
    }
    for (int i = 0; i < points_.size(); ++i) {
      point_by_id_[points_[i].id] = &points_[i];
    }

    json_decref(root);


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


    return true;
  }

  bool SaveJson(const char *dest) {
    json_t *root = json_object();

    // Cameras.
    json_t *cameras = json_object();
    json_object_set(root, "cameras", cameras);
    for (int i = 0; i < cameras_.size(); ++i) {
      json_t *camera = json_object();
      // json_array_append(cameras, camera);
      // json_object_set(camera, "id", json_string(cameras_[i].id.c_str()));
      json_object_set(cameras, cameras_[i].id.c_str(), camera);
      json_object_set(camera, "width", json_real(cameras_[i].width));
      json_object_set(camera, "height", json_real(cameras_[i].height));
      json_object_set(camera, "focal", json_real(cameras_[i].parameters[0]));
      json_object_set(camera, "k1", json_real(cameras_[i].parameters[1]));
      json_object_set(camera, "k2", json_real(cameras_[i].parameters[2]));
    }

    // Shots.
    json_t *shots = json_object();
    json_object_set(root, "shots", shots);
    for (int i = 0; i < shots_.size(); ++i) {
      json_t *shot = json_object();
      // json_array_append(shots, shot);
      // json_object_set(shot, "id", json_string(shots_[i].id.c_str()));
      json_object_set(shots, shots_[i].id.c_str(), shot);
      json_object_set(shot, "camera", json_string(shots_[i].camera.c_str()));
      json_t *Rarray = json_array();
      json_t *tarray = json_array();
      json_object_set(shot, "rotation", Rarray);
      json_object_set(shot, "translation", tarray);
      for (int j = 0; j < 3; ++j)
        json_array_append(Rarray, json_real(shots_[i].parameters[j]));
      for (int j = 0; j < 3; ++j)
        json_array_append(tarray, json_real(shots_[i].parameters[3 + j]));
    }

    // Points.
    json_t *points = json_object();
    json_object_set(root, "points", points);
    for (int i = 0; i < points_.size(); ++i) {
      json_t *point = json_object();
      // json_array_append(points, point);
      // json_object_set(point, "id", json_string(points_[i].id.c_str()));
      json_object_set(points, points_[i].id.c_str(), point);
      json_t *coordinates = json_array();
      json_object_set(point, "coordinates", coordinates);
      for (int j = 0; j < 3; ++j)
        json_array_append(coordinates, json_real(points_[i].parameters[j]));
    }

    json_dump_file(root, dest, JSON_INDENT(4));

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
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

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
    T distortion = T(1.0) + r2  * (l1 + l2  * r2);
    
    // Compute final projected point position.
    const T& focal = camera[0];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  double observed_x;
  double observed_y;
};


class TruncatedLoss : public ceres::LossFunction {
 public:
  explicit TruncatedLoss(double t2)
    : t2_(t2) {
    CHECK_GT(t2, 0.0);
  }

  virtual void Evaluate(double s, double rho[3]) const {
    if (s >= t2_) {
      // Outlier.
      rho[0] = t2_;
      rho[1] = 0.000001;
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
