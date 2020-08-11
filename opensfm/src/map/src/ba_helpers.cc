#include <map/ba_helpers.h>
#include <bundle/bundle_adjuster.h>
#include <map/map.h>
#include <chrono>
#include <map/config.h>
#include <foundation/types.h>
void BAHelpers::SetUpBAFromReconstruction(map::Map& map, BundleAdjuster& ba) {
  ba.SetUseAnalyticDerivatives(true);
  constexpr bool fix_cameras{true};
  // TODO: Check original vs optimized camera?
  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    ba.AddCamera(cam.id, cam, cam, fix_cameras);
  }

  for (const auto& pt_pair : map.GetAllLandmarks()) {
    const auto& pt = pt_pair.second;
    ba.AddPoint(pt.id_, pt.GetGlobalPos(), false);
  }
  constexpr bool bundle_use_gps{true};
  for (const auto& shot_pair : map.GetAllShots()) {
    const auto& shot = shot_pair.second;
    const auto& pose = shot.GetPose();
    ba.AddShot(shot.id_, shot.shot_camera_->id, pose.RotationWorldToCameraMin(),
               pose.TranslationWorldToCamera(), false);
    std::cout << "shot: " << shot.id_ << ", " << pose.RotationWorldToCameraMin() << ", " << pose.TranslationWorldToCamera() << std::endl;
    if (bundle_use_gps) {
      const Vec3d g = shot.shot_measurements_.gps_position_.Value();
      ba.AddPositionPrior(shot.id_, g[0], g[1], g[2],
                          shot.shot_measurements_.gps_accuracy_.Value());
    }
    // Now, divide betwen linear datastructure and map
    if (shot.UseLinearDataStructure()) {
        const auto& keypts = shot.GetKeyPoints();
        const auto& landmarks = shot.GetLandmarks();
        assert(keypts.size() == landmarks.size());
        for (size_t idx = 0; idx < landmarks.size(); ++idx)
        {
            const auto* lm = landmarks[idx];
            if (lm != nullptr)
            {
                //add
                const auto& obs = keypts[idx];
                ba.AddPointProjectionObservation(shot.id_, lm->id_, obs.point[0], obs.point[1], obs.scale);
                std::cout << "point obs" << shot.id_ << ", " << lm->id_ << ", " << obs.point.transpose() << ", " << obs.scale << std::endl;
            }
        }
    }
    else
    {
        for (const auto& lm_obs : shot.GetLandmarkObservations())
        {
            const auto& obs = lm_obs.second;
            ba.AddPointProjectionObservation(shot.id_, lm_obs.first->id_, obs.point[0], obs.point[1], obs.scale);
            std::cout << "point obs" << shot.id_ << ", " << lm_obs.first->id_ << ", " << obs.point.transpose() << ", " << obs.scale << std::endl;

        }
    }
  }
}


struct cmpValue
{
    bool operator()(const std::pair<map::Shot const*, size_t>& val1, const std::pair<map::Shot const*, size_t>& val2) const
            {
              return val1.second > val2.second;
            };
};

/**Reconstructed shots near a given shot.

Returns:
    a tuple with interior and boundary:
    - interior: the list of shots at distance smaller than radius
    - boundary: shots sharing at least on point with the interior

Central shot is at distance 0.  Shots at distance n + 1 share at least
min_common_points points with shots at distance n.
*/
std::pair<std::unordered_set<map::Shot*>, std::unordered_set<map::Shot*>>
BAHelpers::ShotNeighborhood(map::Map& map,
                            const map::ShotId& central_shot_id,
                            const size_t radius, const size_t min_common_points,
                            const size_t max_interior_size) {
  constexpr size_t MaxBoundarySize{1000000};
  std::unordered_set<map::Shot*> interior;
  interior.insert(map.GetShot(central_shot_id));
  for (size_t distance = 1; distance < radius; ++distance)
  {
    if (interior.size() < max_interior_size)
    {
      const auto remaining = max_interior_size - interior.size();
      const auto neighbors = DirectShotNeighbors(map, interior, min_common_points, remaining);
      interior.insert(neighbors.begin(), neighbors.end());
    }
    else
    {
      break;
    }
  }
  const auto boundary = DirectShotNeighbors(map, interior, 1, MaxBoundarySize);
  return std::make_pair(interior, boundary);
}

std::unordered_set<map::Shot*> BAHelpers::DirectShotNeighbors(
    map::Map& map, const std::unordered_set<map::Shot*>& shot_ids,
    const size_t min_common_points, const size_t max_neighbors)
{
  std::unordered_set<map::Landmark*> points;
  for (auto* shot : shot_ids)
  {
    //TODO: implement const GetShot....
    if (shot->UseLinearDataStructure()) {
        const auto& landmarks = shot->GetLandmarks();
        for (size_t idx = 0; idx < landmarks.size(); ++idx)
        {
            auto* lm = landmarks[idx];
            if (lm != nullptr)
            {
              if (!map.HasLandmark(lm->id_))
              {
                std::runtime_error("Valid landmark not in reconstruction!!!");
              }
              points.insert(lm);
            }
        }
    }
    else
    {
        for (const auto& lm_obs : shot->GetLandmarkObservations())
        {
          if (!map.HasLandmark(lm_obs.first->id_))
          {
            std::runtime_error("Valid landmark not in reconstruction!!!");
          }
          points.insert(lm_obs.first);
        }
    }
  }
  std::unordered_set<map::Shot*> candidate_shots;
  for (auto& shot_p : map.GetAllShots()) {
    auto* shot = &shot_p.second;
    if (shot_ids.find(shot) == shot_ids.end()) {
      candidate_shots.insert(shot);
    }
  }

  std::unordered_map<map::Shot*, size_t> common_points;
  for (auto* pt : points)
  {
    for (const auto& neighbor_p : pt->GetObservations())
    {
      auto* shot = neighbor_p.first;
      if (candidate_shots.find(shot) != candidate_shots.end())
      {
        ++common_points[shot];
      }
    }
  }


  std::vector<std::pair<map::Shot*, size_t>> pairs(common_points.begin(), common_points.end());
  std::sort(pairs.begin(), pairs.end(), 
            [](const std::pair<map::Shot*, size_t>& val1, const std::pair<map::Shot*, size_t>& val2)
            {
              return val1.second > val2.second;
            });

  const size_t max_n = std::min(max_neighbors, pairs.size());
  std::unordered_set<map::Shot*> neighbors;
  size_t idx = 0;
  for (auto& p : pairs)
  {
    if (p.second >= min_common_points && idx < max_n)
    {
      neighbors.insert(p.first);
    }
    else
    {
      break;
    }
    ++idx;
  }
  return neighbors;
}


// std::vector<map::LandmarkId>
py::tuple //<std::vector<map::LandmarkId>, py::dict>
BAHelpers::BundleLocal(map::Map& map, const map::ShotId& central_shot_id, const OpenSfMConfig& config)
{
  py::dict report;
  const auto start = std::chrono::high_resolution_clock::now();
  auto neighborhood = ShotNeighborhood(
      map, central_shot_id, config.local_bundle_radius,
      config.local_bundle_min_common_points, config.local_bundle_max_shots);
  auto& interior = neighborhood.first;
  auto& boundary = neighborhood.second;

  //set up BA
  auto ba = BundleAdjuster();
  const bool fix_cameras = !config.optimize_camera_parameters;
  ba.SetUseAnalyticDerivatives(config.bundle_analytic_derivatives);
  
  // TODO: Check original vs optimized camera?
  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    ba.AddCamera(cam.id, cam, cam, fix_cameras);
  }

  //Combine the sets
  std::unordered_set<map::Shot*> int_and_bound(interior.cbegin(), interior.cend());
  int_and_bound.insert(boundary.cbegin(), boundary.cend());
  std::unordered_set<map::Landmark*> points;
  // std::vector<map::LandmarkId> pt_ids;
  py::list pt_ids;
  constexpr bool point_constant{false};
  for (auto* shot : int_and_bound)
  {
    const auto& pose = shot->GetPose();
    const auto shot_constant = boundary.count(shot) > 0;
    ba.AddShot(shot->id_, shot->shot_camera_->id, pose.RotationWorldToCameraMin(),
               pose.TranslationWorldToCamera(), shot_constant);
    if (interior.count(shot) > 0)
    {
      if (config.bundle_use_gps)
      {
        const Vec3d g = shot->shot_measurements_.gps_position_.Value();
        ba.AddPositionPrior(shot->id_, g[0], g[1], g[2],
                            shot->shot_measurements_.gps_accuracy_.Value());
      }
      
      if (shot->UseLinearDataStructure()) {
        const auto& landmarks = shot->GetLandmarks();
        const auto& keypts = shot->GetKeyPoints();
        for (size_t idx = 0; idx < landmarks.size(); ++idx)
        {
          auto* lm = landmarks[idx];
          if (lm != nullptr)
          {
            if (!map.HasLandmark(lm->id_))
            {
              std::runtime_error("Valid landmark not in reconstruction!!!");
            }
            if(points.count(lm) == 0)
            {
              points.insert(lm);
              pt_ids.append(lm->id_);
              ba.AddPoint(lm->id_, lm->GetGlobalPos(), point_constant);
            }
            const auto& obs = keypts[idx];
            ba.AddPointProjectionObservation(shot->id_, lm->id_, obs.point[0], obs.point[1], obs.scale);
          }
        }
      }
      else
      {
        for (const auto& lm_obs : shot->GetLandmarkObservations())
        {
          if (!map.HasLandmark(lm_obs.first->id_))
          {
            std::runtime_error("Valid landmark not in reconstruction!!!");
          }
          auto* lm = lm_obs.first;
          if(points.count(lm) == 0)
          {
            points.insert(lm);
            pt_ids.append(lm->id_);
            ba.AddPoint(lm->id_, lm->GetGlobalPos(), point_constant);
          }
          const auto& obs = lm_obs.second;
          ba.AddPointProjectionObservation(shot->id_, lm_obs.first->id_, obs.point[0], obs.point[1], obs.scale);
        }
      }
    }
  }

  ba.SetPointProjectionLossFunction(config.loss_function,
                                    config.loss_function_threshold);
  ba.SetInternalParametersPriorSD(
      config.exif_focal_sd, config.principal_point_sd,
      config.radial_distorsion_k1_sd, config.radial_distorsion_k2_sd,
      config.radial_distorsion_p1_sd, config.radial_distorsion_p2_sd,
      config.radial_distorsion_k3_sd);

  ba.SetNumThreads(config.processes);
  ba.SetMaxNumIterations(10);
  ba.SetLinearSolverType("DENSE_SCHUR");
  const auto timer_setup = std::chrono::high_resolution_clock::now();
  ba.Run();
  const auto timer_run = std::chrono::high_resolution_clock::now();
  report["brief_report"] = ba.BriefReport();
  for (auto* shot : interior)
  {
    const auto& s = ba.GetShot(shot->id_);
    shot->GetPose().SetFromWorldToCamera(s.GetRotation(), s.GetTranslation());
  }

  for (auto* point : points)
  {
    const auto& pt = ba.GetPoint(point->id_);
    point->SetGlobalPos(pt.GetPoint());
    point->SetReprojectionErrors(pt.reprojection_errors);
  }
  const auto timer_teardown = std::chrono::high_resolution_clock::now();
  report["wall_times"] = py::dict();
  report["wall_times"]["setup"] = std::chrono::duration_cast<std::chrono::microseconds>(timer_setup - start).count()/1000000.0;
  report["wall_times"]["run"] = std::chrono::duration_cast<std::chrono::microseconds>(timer_run - timer_setup).count()/1000000.0;
  report["wall_times"]["teardown"] = std::chrono::duration_cast<std::chrono::microseconds>(timer_teardown - timer_run).count()/1000000.0;
  report["num_interior_images"] = interior.size();
  report["num_boundary_images"] = boundary.size();
  report["num_other_images"] = map.NumberOfShots() - interior.size() - boundary.size();
  return py::make_tuple(pt_ids, report);
}


py::dict
BAHelpers::Bundle(map::Map& map, const OpenSfMConfig& config)
{
  py::dict report;
  report["wall_times"] = py::dict();
  auto ba = BundleAdjuster();
  const bool fix_cameras = !config.optimize_camera_parameters;
  ba.SetUseAnalyticDerivatives(config.bundle_analytic_derivatives);
  const auto start = std::chrono::high_resolution_clock::now();

  // TODO: Check original vs optimized camera?
  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    ba.AddCamera(cam.id, cam, cam, fix_cameras);
  }

  for (const auto& pt_pair : map.GetAllLandmarks()) {
    const auto& pt = pt_pair.second;
    ba.AddPoint(pt.id_, pt.GetGlobalPos(), false);
    std::cout << "point: " << pt.id_ << ", " << pt.GetGlobalPos().transpose() << std::endl;
  }
  
  //Alignemnt method
  //TODO: Alignment method, auto and vertical
  const auto& align_method = config.align_method;
  if (align_method.compare("auto") == 0)
  {
    std::runtime_error("Implement Auto alignment");
  }
  bool do_add_align_vector = false;
  Vec3d up_vector(0);
  if (align_method.compare("orientation_prior") == 0)
  {
    if (config.align_orientation_prior.compare("vertical") == 0)
    {
      do_add_align_vector = true;
      up_vector = Vec3d(0, 0, -1);
    } 
    else if (config.align_orientation_prior.compare("vertical") == 0) 
    {
      do_add_align_vector = true;
      up_vector = Vec3d(0, -1, 0);
    }
  }


  for (const auto& shot_pair : map.GetAllShots()) {
    const auto& shot = shot_pair.second;
    const auto& pose = shot.GetPose();
    constexpr auto fix_shot = false;
    ba.AddShot(shot.id_, shot.shot_camera_->id, pose.RotationWorldToCameraMin(),
               pose.TranslationWorldToCamera(), fix_shot);
    std::cout << "shot: " << shot.id_ << ", " << pose.RotationWorldToCameraMin().transpose() << ", " << pose.TranslationWorldToCamera().transpose() << std::endl;
    if (config.bundle_use_gps) {
      const Vec3d g = shot.shot_measurements_.gps_position_.Value();
      ba.AddPositionPrior(shot.id_, g[0], g[1], g[2],
                          shot.shot_measurements_.gps_accuracy_.Value());
      std::cout << "pos prior: " << g.transpose() << std::endl;
    }

    if (do_add_align_vector)
    {
      constexpr double std_dev = 1e-3;
      ba.AddAbsoluteUpVector(shot.id_, up_vector, std_dev);
    }
    // Now, divide betwen linear datastructure and map
    if (shot.UseLinearDataStructure()) {
        const auto& keypts = shot.GetKeyPoints();
        const auto& landmarks = shot.GetLandmarks();
        assert(keypts.size() == landmarks.size());
        for (size_t idx = 0; idx < landmarks.size(); ++idx)
        {
            const auto* lm = landmarks[idx];
            if (lm != nullptr)
            {
                //add
                const auto& obs = keypts[idx];
                ba.AddPointProjectionObservation(shot.id_, lm->id_, obs.point[0], obs.point[1], obs.scale);
                std::cout << "point, obs:  " << shot.id_ << " " << lm->id_ << " " << obs.point.transpose() << " " << obs.scale << std::endl;
            }
        }
    }
    else
    {
        for (const auto& lm_obs : shot.GetLandmarkObservations())
        {
            const auto& obs = lm_obs.second;
            ba.AddPointProjectionObservation(shot.id_, lm_obs.first->id_, obs.point[0], obs.point[1], obs.scale);
            std::cout << "point obs" << shot.id_ << ", " << lm_obs.first->id_ << ", " << obs.point.transpose() << ", " << obs.scale << std::endl;
        }
    }
  }
    //TODO: GCP
    if (config.bundle_use_gcp)
    {
      std::runtime_error("TODO: Implement GCP");
    }
    // if (config.bundle_use_gcp && !gcp.empty())
    // {
    //   ba.
    // }
    
    ba.SetPointProjectionLossFunction(config.loss_function,
                                      config.loss_function_threshold);
    ba.SetInternalParametersPriorSD(
        config.exif_focal_sd, config.principal_point_sd,
        config.radial_distorsion_k1_sd, config.radial_distorsion_k2_sd,
        config.radial_distorsion_p1_sd, config.radial_distorsion_p2_sd,
        config.radial_distorsion_k3_sd);
    
    ba.SetNumThreads(config.processes);
    ba.SetMaxNumIterations(config.bundle_max_iterations);
    ba.SetLinearSolverType("SPARSE_SCHUR");
    const auto timer_setup = std::chrono::high_resolution_clock::now();
    ba.Run();
    const auto timer_run = std::chrono::high_resolution_clock::now();
    report["brief_report"] = ba.BriefReport();
    std::cout << "ba: " << ba.BriefReport() << std::endl;
    // update cameras if optimized
    if (!fix_cameras)
    {
      for (auto& cam : map.GetCameras())
      {
        const auto& ba_cam = ba.GetCamera(cam.first);
        for (const auto& p : ba_cam.GetParametersMap())
        {
          cam.second.SetParameterValue(p.first, p.second);
        }
      }
    }

    //Update shots
    for (auto& shot : map.GetAllShots())
    {
      const auto& s = ba.GetShot(shot.first);
      shot.second.GetPose().SetFromWorldToCamera(s.GetRotation(), s.GetTranslation());
    }

    //Update points
    for (auto& point : map.GetAllLandmarks())
    {
      const auto& pt = ba.GetPoint(point.first);
      point.second.SetGlobalPos(pt.GetPoint());
      point.second.SetReprojectionErrors(pt.reprojection_errors);
    }
    const auto timer_teardown = std::chrono::high_resolution_clock::now();
    report["wall_times"] = py::dict();
    report["wall_times"]["setup"] = std::chrono::duration_cast<std::chrono::microseconds>(timer_setup - start).count()/1000000.0;
    report["wall_times"]["run"] = std::chrono::duration_cast<std::chrono::microseconds>(timer_run - timer_setup).count()/1000000.0;
    report["wall_times"]["teardown"] = std::chrono::duration_cast<std::chrono::microseconds>(timer_teardown - timer_run).count()/1000000.0;
    return report;
}
