#include <bundle/bundle_adjuster.h>
#include <foundation/types.h>
#include <geometry/triangulation.h>
#include <map/ba_helpers.h>
#include <map/ground_control_points.h>
#include <map/map.h>

#include <chrono>

std::pair<std::unordered_set<map::ShotId>, std::unordered_set<map::ShotId>>
BAHelpers::ShotNeighborhoodIds(map::Map& map,
                               const map::ShotId& central_shot_id,
                               size_t radius, size_t min_common_points,
                               size_t max_interior_size) {
  auto res = ShotNeighborhood(map, central_shot_id, radius, min_common_points,
                              max_interior_size);
  std::unordered_set<map::ShotId> interior;
  for (map::Shot* shot : res.first) {
    interior.insert(shot->GetId());
  }
  std::unordered_set<map::ShotId> boundary;
  for (map::Shot* shot : res.second) {
    boundary.insert(shot->GetId());
  }
  return std::make_pair(interior, boundary);
}

/**Reconstructed shots near a given shot.

Returns:
    a tuple with interior and boundary:
    - interior: the list of shots at distance smaller than radius
    - boundary: shots sharing at least on point with the interior

Central shot is at distance 0.  Shots at distance n + 1 share at least
min_common_points points with shots at distance n.
*/
std::pair<std::unordered_set<map::Shot*>, std::unordered_set<map::Shot*>>
BAHelpers::ShotNeighborhood(map::Map& map, const map::ShotId& central_shot_id,
                            size_t radius, size_t min_common_points,
                            size_t max_interior_size) {
  constexpr size_t MaxBoundarySize{1000000};
  std::unordered_set<map::Shot*> interior;
  interior.insert(&map.GetShot(central_shot_id));
  for (size_t distance = 1;
       distance < radius && interior.size() < max_interior_size; ++distance) {
    const auto remaining = max_interior_size - interior.size();
    const auto neighbors =
        DirectShotNeighbors(interior, min_common_points, remaining);
    interior.insert(neighbors.begin(), neighbors.end());
  }

  const auto boundary = DirectShotNeighbors(interior, 1, MaxBoundarySize);
  return std::make_pair(interior, boundary);
}

std::unordered_set<map::Shot*> BAHelpers::DirectShotNeighbors(
    const std::unordered_set<map::Shot*>& shot_ids,
    const size_t min_common_points, const size_t max_neighbors) {
  std::unordered_set<map::Landmark*> points;
  for (auto* shot : shot_ids) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      points.insert(lm_obs.first);
    }
  }

  std::unordered_map<map::Shot*, size_t> common_points;
  for (auto* pt : points) {
    for (const auto& neighbor_p : pt->GetObservations()) {
      auto* shot = neighbor_p.first;
      if (shot_ids.find(shot) == shot_ids.end()) {
        ++common_points[shot];
      }
    }
  }

  std::vector<std::pair<map::Shot*, size_t>> pairs(common_points.begin(),
                                                   common_points.end());
  std::sort(pairs.begin(), pairs.end(),
            [](const std::pair<map::Shot*, size_t>& val1,
               const std::pair<map::Shot*, size_t>& val2) {
              return val1.second > val2.second;
            });

  const size_t max_n = std::min(max_neighbors, pairs.size());
  std::unordered_set<map::Shot*> neighbors;
  size_t idx = 0;
  for (auto& p : pairs) {
    if (p.second >= min_common_points && idx < max_n) {
      neighbors.insert(p.first);
    } else {
      break;
    }
    ++idx;
  }
  return neighbors;
}

py::tuple BAHelpers::BundleLocal(
    map::Map& map,
    const std::unordered_map<map::CameraId, Camera>& camera_priors,
    const AlignedVector<map::GroundControlPoint>& gcp,
    const map::ShotId& central_shot_id, const py::dict& config) {
  py::dict report;
  const auto start = std::chrono::high_resolution_clock::now();
  auto neighborhood = ShotNeighborhood(
      map, central_shot_id, config["local_bundle_radius"].cast<size_t>(),
      config["local_bundle_min_common_points"].cast<size_t>(),
      config["local_bundle_max_shots"].cast<size_t>());
  auto& interior = neighborhood.first;
  auto& boundary = neighborhood.second;

  // set up BA
  auto ba = BundleAdjuster();
  ba.SetUseAnalyticDerivatives(
      config["bundle_analytic_derivatives"].cast<bool>());

  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    const auto& cam_prior = camera_priors.at(cam.id);
    constexpr bool fix_cameras{true};
    ba.AddCamera(cam.id, cam, cam_prior, fix_cameras);
  }
  // Combine the sets
  std::unordered_set<map::Shot*> int_and_bound(interior.cbegin(),
                                               interior.cend());
  int_and_bound.insert(boundary.cbegin(), boundary.cend());
  std::unordered_set<map::Landmark*> points;
  py::list pt_ids;

  constexpr bool point_constant{false};

  // Add interior shots
  for (auto* shot : interior) {
    const auto& pose = shot->GetPose();
    constexpr auto shot_constant{false};

    ba.AddShot(shot->id_, shot->GetCamera()->id,
               pose->RotationWorldToCameraMin(),
               pose->TranslationWorldToCamera(), shot_constant);
    if (config["bundle_use_gps"].cast<bool>()) {
      const Vec3d g = shot->GetShotMeasurements().gps_position_.Value();
      ba.AddPositionPrior(shot->id_, g[0], g[1], g[2],
                          shot->GetShotMeasurements().gps_accuracy_.Value());
    }
  }

  // add boundary shots
  for (auto* shot : boundary) {
    const auto& pose = shot->GetPose();
    constexpr auto shot_constant{true};
    ba.AddShot(shot->id_, shot->GetCamera()->id,
               pose->RotationWorldToCameraMin(),
               pose->TranslationWorldToCamera(), shot_constant);
  }

  for (auto* shot : interior) {
    // Add all points of the shots that are in the interior
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      auto* lm = lm_obs.first;
      if (points.count(lm) == 0) {
        points.insert(lm);
        pt_ids.append(lm->id_);
        ba.AddPoint(lm->id_, lm->GetGlobalPos(), point_constant);
      }
      const auto& obs = lm_obs.second;
      ba.AddPointProjectionObservation(shot->id_, lm_obs.first->id_,
                                       obs.point[0], obs.point[1], obs.scale);
    }
  }
  for (auto* shot : boundary) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      auto* lm = lm_obs.first;
      if (points.count(lm) > 0) {
        const auto& obs = lm_obs.second;
        ba.AddPointProjectionObservation(shot->id_, lm_obs.first->id_,
                                         obs.point[0], obs.point[1], obs.scale);
      }
    }
  }

  if (config["bundle_use_gcp"].cast<bool>() && !gcp.empty()) {
    AddGCPToBundle(ba, gcp, map.GetShots());
  }

  ba.SetPointProjectionLossFunction(
      config["loss_function"].cast<std::string>(),
      config["loss_function_threshold"].cast<double>());
  ba.SetInternalParametersPriorSD(
      config["exif_focal_sd"].cast<double>(),
      config["principal_point_sd"].cast<double>(),
      config["radial_distortion_k1_sd"].cast<double>(),
      config["radial_distortion_k2_sd"].cast<double>(),
      config["tangential_distortion_p1_sd"].cast<double>(),
      config["tangential_distortion_p2_sd"].cast<double>(),
      config["radial_distortion_k3_sd"].cast<double>(),
      config["radial_distortion_k4_sd"].cast<double>());

  ba.SetNumThreads(config["processes"].cast<int>());
  ba.SetMaxNumIterations(10);
  ba.SetLinearSolverType("DENSE_SCHUR");
  const auto timer_setup = std::chrono::high_resolution_clock::now();

  {
    py::gil_scoped_release release;
    ba.Run();
  }

  const auto timer_run = std::chrono::high_resolution_clock::now();
  for (auto* shot : interior) {
    const auto& s = ba.GetShot(shot->id_);
    shot->GetPose()->SetFromWorldToCamera(s.GetRotation(), s.GetTranslation());
  }

  for (auto* point : points) {
    const auto& pt = ba.GetPoint(point->id_);
    point->SetGlobalPos(pt.GetPoint());
    point->SetReprojectionErrors(pt.reprojection_errors);
  }
  const auto timer_teardown = std::chrono::high_resolution_clock::now();
  report["brief_report"] = ba.BriefReport();
  report["wall_times"] = py::dict();
  report["wall_times"]["setup"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_setup - start)
          .count() /
      1000000.0;
  report["wall_times"]["run"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_run -
                                                            timer_setup)
          .count() /
      1000000.0;
  report["wall_times"]["teardown"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_teardown -
                                                            timer_run)
          .count() /
      1000000.0;
  report["num_interior_images"] = interior.size();
  report["num_boundary_images"] = boundary.size();
  report["num_other_images"] =
      map.NumberOfShots() - interior.size() - boundary.size();
  return py::make_tuple(pt_ids, report);
}

bool BAHelpers::TriangulateGCP(
    const map::GroundControlPoint& point,
    const std::unordered_map<map::ShotId, map::Shot>& shots,
    Vec3d& coordinates) {
  constexpr auto reproj_threshold{1.0};
  constexpr auto min_ray_angle = 0.1 * M_PI / 180.0;
  MatX3d os, bs;
  size_t added = 0;
  coordinates = Vec3d::Zero();
  bs.conservativeResize(point.observations_.size(), Eigen::NoChange);
  os.conservativeResize(point.observations_.size(), Eigen::NoChange);
  for (const auto& obs : point.observations_) {
    const auto shot_it = shots.find(obs.shot_id_);
    if (shot_it != shots.end()) {
      const auto& shot = (shot_it->second);
      const Vec3d bearing = shot.GetCamera()->Bearing(obs.projection_);
      const auto& shot_pose = shot.GetPose();
      bs.row(added) = shot_pose->RotationCameraToWorld() * bearing;
      os.row(added) = shot_pose->GetOrigin();
      ++added;
    }
  }
  bs.conservativeResize(added, Eigen::NoChange);
  os.conservativeResize(added, Eigen::NoChange);
  if (added >= 2) {
    const std::vector<double> thresholds(added, reproj_threshold);
    const auto& res = geometry::TriangulateBearingsMidpoint(os, bs, thresholds,
                                                            min_ray_angle);
    coordinates = res.second;
    return res.first;
  }
  return false;
}

// Add Ground Control Points constraints to the bundle problem
void BAHelpers::AddGCPToBundle(
    BundleAdjuster& ba, const AlignedVector<map::GroundControlPoint>& gcp,
    const std::unordered_map<map::ShotId, map::Shot>& shots) {
  for (const auto& point : gcp) {
    const auto point_id = "gcp-" + point.id_;
    Vec3d coordinates;
    if (!TriangulateGCP(point, shots, coordinates)) {
      if (point.coordinates_.HasValue()) {
        coordinates = point.coordinates_.Value();
      } else {
        continue;
      }
    }
    constexpr auto point_constant{false};
    ba.AddPoint(point_id, coordinates, point_constant);
    if (point.coordinates_.HasValue()) {
      const auto point_type = point.has_altitude_ ? PositionConstraintType::XYZ
                                                  : PositionConstraintType::XY;
      ba.AddPointPositionWorld(point_id, point.coordinates_.Value(), 0.1,
                               point_type);
    }

    // Now iterate through the observations
    for (const auto& obs : point.observations_) {
      const auto& shot_id = obs.shot_id_;
      if (shots.count(shot_id) > 0) {
        constexpr double scale{0.0001};
        ba.AddPointProjectionObservation(shot_id, point_id, obs.projection_[0],
                                         obs.projection_[1], scale);
      }
    }
  }
}

py::dict BAHelpers::Bundle(
    map::Map& map,
    const std::unordered_map<map::CameraId, Camera>& camera_priors,
    const AlignedVector<map::GroundControlPoint>& gcp, const py::dict& config) {
  py::dict report;

  auto ba = BundleAdjuster();
  const bool fix_cameras = !config["optimize_camera_parameters"].cast<bool>();
  ba.SetUseAnalyticDerivatives(
      config["bundle_analytic_derivatives"].cast<bool>());
  const auto start = std::chrono::high_resolution_clock::now();

  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    const auto& cam_prior = camera_priors.at(cam.id);
    ba.AddCamera(cam.id, cam, cam_prior, fix_cameras);
  }

  for (const auto& pt_pair : map.GetLandmarks()) {
    const auto& pt = pt_pair.second;
    ba.AddPoint(pt.id_, pt.GetGlobalPos(), false);
  }

  auto align_method = config["align_method"].cast<std::string>();
  if (align_method.compare("auto") == 0) {
    align_method = DetectAlignmentConstraints(map, config, gcp);
  }
  bool do_add_align_vector = false;
  Vec3d up_vector = Vec3d::Zero();
  if (align_method.compare("orientation_prior") == 0) {
    const std::string align_orientation_prior =
        config["align_orientation_prior"].cast<std::string>();
    if (align_orientation_prior.compare("vertical") == 0) {
      do_add_align_vector = true;
      up_vector = Vec3d(0, 0, -1);
    } else if (align_orientation_prior.compare("horizontal") == 0) {
      do_add_align_vector = true;
      up_vector = Vec3d(0, -1, 0);
    }
  }

  for (const auto& shot_pair : map.GetShots()) {
    const auto& shot = shot_pair.second;
    const auto& pose = shot.GetPose();
    constexpr auto fix_shot = false;
    ba.AddShot(shot.id_, shot.GetCamera()->id, pose->RotationWorldToCameraMin(),
               pose->TranslationWorldToCamera(), fix_shot);
    if (config["bundle_use_gps"].cast<bool>()) {
      const Vec3d g = shot.GetShotMeasurements().gps_position_.Value();
      ba.AddPositionPrior(shot.id_, g[0], g[1], g[2],
                          shot.GetShotMeasurements().gps_accuracy_.Value());
    }

    if (do_add_align_vector) {
      constexpr double std_dev = 1e-3;
      ba.AddAbsoluteUpVector(shot.id_, up_vector, std_dev);
    }
    for (const auto& lm_obs : shot.GetLandmarkObservations()) {
      const auto& obs = lm_obs.second;
      ba.AddPointProjectionObservation(shot.id_, lm_obs.first->id_,
                                       obs.point[0], obs.point[1], obs.scale);
    }
  }
  if (config["bundle_use_gcp"].cast<bool>() && !gcp.empty()) {
    AddGCPToBundle(ba, gcp, map.GetShots());
  }

  ba.SetPointProjectionLossFunction(
      config["loss_function"].cast<std::string>(),
      config["loss_function_threshold"].cast<double>());
  ba.SetInternalParametersPriorSD(
      config["exif_focal_sd"].cast<double>(),
      config["principal_point_sd"].cast<double>(),
      config["radial_distortion_k1_sd"].cast<double>(),
      config["radial_distortion_k2_sd"].cast<double>(),
      config["tangential_distortion_p1_sd"].cast<double>(),
      config["tangential_distortion_p2_sd"].cast<double>(),
      config["radial_distortion_k3_sd"].cast<double>(),
      config["radial_distortion_k4_sd"].cast<double>());

  ba.SetNumThreads(config["processes"].cast<int>());
  ba.SetMaxNumIterations(config["bundle_max_iterations"].cast<int>());
  ba.SetLinearSolverType("SPARSE_SCHUR");
  const auto timer_setup = std::chrono::high_resolution_clock::now();

  {
    py::gil_scoped_release release;
    ba.Run();
  }

  const auto timer_run = std::chrono::high_resolution_clock::now();
  // update cameras if optimized
  if (!fix_cameras) {
    for (auto& cam : map.GetCameras()) {
      const auto& ba_cam = ba.GetCamera(cam.first);
      for (const auto& p : ba_cam.GetParametersMap()) {
        cam.second.SetParameterValue(p.first, p.second);
      }
    }
  }

  // Update shots
  for (auto& shot : map.GetShots()) {
    const auto& s = ba.GetShot(shot.first);
    shot.second.GetPose()->SetFromWorldToCamera(s.GetRotation(),
                                                s.GetTranslation());
  }

  // Update points
  for (auto& point : map.GetLandmarks()) {
    const auto& pt = ba.GetPoint(point.first);
    point.second.SetGlobalPos(pt.GetPoint());
    point.second.SetReprojectionErrors(pt.reprojection_errors);
  }
  const auto timer_teardown = std::chrono::high_resolution_clock::now();
  report["brief_report"] = ba.BriefReport();
  report["wall_times"] = py::dict();
  report["wall_times"]["setup"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_setup - start)
          .count() /
      1000000.0;
  report["wall_times"]["run"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_run -
                                                            timer_setup)
          .count() /
      1000000.0;
  report["wall_times"]["teardown"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_teardown -
                                                            timer_run)
          .count() /
      1000000.0;
  return report;
}

void BAHelpers::AlignmentConstraints(
    const map::Map& map, const py::dict& config,
    const AlignedVector<map::GroundControlPoint>& gcp, MatX3d& Xp, MatX3d& X) {
  size_t reserve_size = 0;
  if (!gcp.empty() && config["bundle_use_gcp"].cast<bool>()) {
    reserve_size += gcp.size();
  }
  if (config["bundle_use_gps"].cast<bool>()) {
    reserve_size += map.NumberOfShots();
  }
  Xp.conservativeResize(reserve_size, Eigen::NoChange);
  X.conservativeResize(reserve_size, Eigen::NoChange);
  size_t idx = 0;
  const auto& shots = map.GetShots();
  // Triangulated vs measured points
  if (!gcp.empty() && config["bundle_use_gcp"].cast<bool>()) {
    for (const auto& point : gcp) {
      Vec3d coordinates;
      if (TriangulateGCP(point, shots, coordinates)) {
        Xp.row(idx) = point.coordinates_.Value();
        X.row(idx) = coordinates;
        ++idx;
      }
    }
  }
  if (config["bundle_use_gps"].cast<bool>()) {
    for (const auto& shot_p : shots) {
      const auto& shot = shot_p.second;
      Xp.row(idx) = shot.GetShotMeasurements().gps_position_.Value();
      X.row(idx) = shot.GetPose()->GetOrigin();
      ++idx;
    }
  }
}

std::string BAHelpers::DetectAlignmentConstraints(
    const map::Map& map, const py::dict& config,
    const AlignedVector<map::GroundControlPoint>& gcp) {
  MatX3d X, Xp;
  AlignmentConstraints(map, config, gcp, Xp, X);
  if (X.rows() < 3) {
    return "orientation_prior";
  }
  const Vec3d X_mean = X.colwise().mean();
  const MatX3d X_zero = X.rowwise() - X_mean.transpose();
  const Mat3d input = X_zero.transpose() * X_zero;
  Eigen::SelfAdjointEigenSolver<MatXd> ses(input, Eigen::EigenvaluesOnly);
  const Vec3d evals = ses.eigenvalues();
  const auto ratio_1st_2nd = std::abs(evals[2] / evals[1]);
  constexpr double epsilon_abs = 1e-10;
  constexpr double epsilon_ratio = 5e3;
  int cond1 = 0;
  for (int i = 0; i < 3; ++i) {
    cond1 += (evals[i] < epsilon_abs) ? 1 : 0;
  }
  const bool is_line = cond1 > 1 || ratio_1st_2nd > epsilon_ratio;
  if (is_line) {
    return "orientation_prior";
  }

  return "naive";
}
