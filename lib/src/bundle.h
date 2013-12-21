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


// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
 public:
  ~BALProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations()       const { return num_observations_;               }
  const double* observations() const { return observations_;                   }
  double* mutable_cameras()          { return parameters_;                     }
  double* mutable_points()           { return parameters_  + 9 * num_cameras_; }

  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 9;
  }
  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }


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
    // Count cameras and points
    //////////////////////////////////////////////////////////////////
    json_t *cameras = json_object_get(root, "cameras");
    if(!json_is_array(cameras)) {
        fprintf(stderr, "error: cameras is not an array\n");
        json_decref(root);
        return false;
    }
    num_cameras_ = json_array_size(cameras);

    json_t *points = json_object_get(root, "points");
    if(!json_is_array(points)) {
        fprintf(stderr, "error: points is not an array\n");
        json_decref(root);
        return false;
    }
    num_points_ = json_array_size(points);
    
    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];


    //////////////////////////////////////////////////////////////////
    // Read cameras
    //////////////////////////////////////////////////////////////////
    for (int i = 0; i < num_cameras_; ++i) {
      json_t *camera = json_array_get(cameras, i);
      const char *camera_id = json_string_value(json_object_get(camera, "id"));
      double width = json_number_value(json_object_get(camera, "width"));
      double height = json_number_value(json_object_get(camera, "height"));
      double focal = json_number_value(json_object_get(camera, "focal"));
      double k1 = json_number_value(json_object_get(camera, "k1"));
      double k2 = json_number_value(json_object_get(camera, "k2"));
      json_t *Rarray = json_object_get(camera, "rotation");
      json_t *tarray = json_object_get(camera, "translation");
      double R[3], t[3];
      for (int j = 0; j < 3; ++j)
        R[j] = json_number_value(json_array_get(Rarray, j));
      for (int j = 0; j < 3; ++j)
        t[j] = json_number_value(json_array_get(tarray, j));

      camera_ids_.push_back(camera_id);
      camera_index_by_id_[camera_id] = i;
      width_[camera_id] = width;
      height_[camera_id] = height;

      double *p = mutable_cameras() + 9 * i;
      p[0] = R[0];
      p[1] = R[1];
      p[2] = R[2];
      p[3] = t[0];
      p[4] = t[1];
      p[5] = t[2];
      p[6] = focal;
      p[7] = k1;
      p[8] = k2;
    }

    //////////////////////////////////////////////////////////////////
    // Read points
    //////////////////////////////////////////////////////////////////
    for (int i = 0; i < num_points_; ++i) {
      json_t *point = json_array_get(points, i);
      const char *point_id = json_string_value(json_object_get(point, "id"));
      json_t *coordinates = json_object_get(point, "coordinates");
      point_ids_.push_back(point_id);
      point_index_by_id_[point_id] = i;

      double *p = mutable_points() + 3 * i;      
      for (int j = 0; j < 3; ++j)
        p[j] = json_number_value(json_array_get(coordinates, j));
    }

    json_decref(root);


    //////////////////////////////////////////////////////////////////
    // Read observations
    //////////////////////////////////////////////////////////////////
    FILE* fptr = fopen(tracks, "r");
    if (fptr == NULL) {
      return false;
    };

    num_observations_ = 0;
    while (true) {
      char camera_id[1000];
      char point_id[1000];
      int oid;
      double x, y;
      int n = fscanf(fptr, "%s %s %d %lg %lg", camera_id, point_id, &oid, &x, &y);
      if (n != 5) break;
      num_observations_++;
    }
    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    rewind(fptr);
    for (int i = 0; i < num_observations_; ++i) {
      char camera_id[1000];
      char point_id[1000];
      int oid;
      double x, y;
      int n = fscanf(fptr, "%s %s %d %lg %lg", camera_id, point_id, &oid, &x, &y);
      camera_index_[i] = camera_index_by_id_[camera_id];
      point_index_[i] = point_index_by_id_[point_id];
      observations_[2 * i + 0] = x - width_[camera_id] / 2;
      observations_[2 * i + 1] = y - height_[camera_id] / 2;
    }

    return true;
  }

  bool SaveJson(const char *dest) {
    json_t *root = json_object();

    // Cameras.
    json_t *cameras = json_array();
    json_object_set(root, "cameras", cameras);
    for (int i = 0; i < num_cameras_; ++i) {
      double *p = mutable_cameras() + 9 * i;
      json_t *camera = json_object();
      json_array_append(cameras, camera);
      json_object_set(camera, "id", json_string(camera_ids_[i].c_str()));

      json_object_set(camera, "width", json_real(width_[camera_ids_[i]]));
      json_object_set(camera, "height", json_real(height_[camera_ids_[i]]));
      json_object_set(camera, "focal", json_real(p[6]));
      json_object_set(camera, "k1", json_real(p[7]));
      json_object_set(camera, "k2", json_real(p[8]));

      json_t *Rarray = json_array();
      json_t *tarray = json_array();
      json_object_set(camera, "rotation", Rarray);
      json_object_set(camera, "translation", Rarray);
      for (int j = 0; j < 3; ++j)
        json_array_append(Rarray, json_real(p[j]));
      for (int j = 0; j < 3; ++j)
        json_array_append(tarray, json_real(p[3 + j]));
    }

    // Points.
    json_t *points = json_array();
    json_object_set(root, "points", points);
    for (int i = 0; i < num_points_; ++i) {
      double *p = mutable_points() + 3 * i;      
      json_t *point = json_object();
      json_array_append(points, point);
      json_object_set(point, "id", json_string(point_ids_[i].c_str()));
      json_t *coordinates = json_array();
      json_object_set(point, "coordinates", coordinates);
      for (int j = 0; j < 3; ++j)
        json_array_append(coordinates, json_real(p[j]));
    }

    json_dump_file(root, dest, JSON_INDENT(4));
    return true;
  }


 private:
  template<typename T>
  void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }

  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;

  std::vector<std::string> camera_ids_;
  std::map<std::string, size_t> camera_index_by_id_;
  std::map<std::string, double> width_, height_; // principal point
  std::vector<std::string> point_ids_;
  std::map<std::string, size_t> point_index_by_id_;
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    const T& focal = camera[6];
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
