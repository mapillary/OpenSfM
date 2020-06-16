
#include <slam/slam_utilities.h>
#include <map/shot.h>
#include <map/landmark.h>
#include <geometry/camera.h>
#include <geometry/triangulation.h>
#include <unordered_set>
#include <unordered_map>
namespace slam
{

// Mat3d
Mat3d
SlamUtilities::to_skew_symmetric_mat(const Vec3d &vec)
{
  Mat3d skew;
  skew << 0, -vec(2), vec(1),
      vec(2), 0, -vec(0),
      -vec(1), vec(0), 0;
  return skew;
}

Mat3d
SlamUtilities::create_E_21(const Mat3d &rot_1w, const Vec3d &trans_1w,
                           const Mat3d &rot_2w, const Vec3d &trans_2w)
{
  const Mat3d rot_21 = rot_2w * rot_1w.transpose();
  const Vec3d trans_21 = -rot_21 * trans_1w + trans_2w;
  const Mat3d trans_21_x = to_skew_symmetric_mat(trans_21);
  return trans_21_x * rot_21;
}

MatXd
SlamUtilities::ConvertOpenCVKptsToEigen(const AlignedVector<Observation>& keypts)
{
  if (!keypts.empty())
  {
    const auto n_kpts = keypts.size();
    MatXd mat(n_kpts, 5);
    for (size_t i = 0; i < n_kpts; ++i)
    {
      const auto &kpt = keypts[i];
      mat.row(i) << kpt.point[0], kpt.point[1], kpt.scale, kpt.angle, kpt.octave;
    }
    return mat;
  }
  return MatXd();
}

// std::vector<map::Landmark *>
// SlamUtilities::update_local_landmarks(const std::vector<map::Shot *> &local_keyframes)
// {
//   std::unordered_set<map::Landmark *> local_landmarks;
//   for (auto keyframe : local_keyframes)
//   {
//     for (auto lm : keyframe->GetLandmarks())
//     {
//       if (lm != nullptr)
//       {
//         local_landmarks.emplace(lm);
//       }
//     }
//   }
//   return std::vector<map::Landmark *>(local_landmarks.begin(), local_landmarks.end());
// }

// std::vector<map::Shot *>
// SlamUtilities::update_local_keyframes(const map::Shot &curr_shot)
// {
//   constexpr unsigned int max_num_local_keyfrms{60};

//   // count the number of sharing landmarks between the current frame and each of the neighbor keyframes
//   // key: keyframe, value: number of sharing landmarks
//   std::unordered_map<map::Shot *, unsigned int> keyfrm_weights;
//   const auto &landmarks = curr_shot.GetLandmarks();
//   const auto n_keypts = landmarks.size();
//   for (unsigned int idx = 0; idx < n_keypts; ++idx)
//   {
//     auto lm = landmarks.at(idx);
//     if (lm != nullptr)
//     {
//       const auto &observations = lm->GetObservations();
//       for (const auto &obs : observations)
//       {
//         ++keyfrm_weights[obs.first];
//       }
//     }
//   }

//   if (keyfrm_weights.empty())
//   {
//     return std::vector<map::Shot *>();
//   }

//   // set the aforementioned keyframes as local keyframes
//   // and find the nearest keyframe
//   unsigned int max_weight = 0;

//   //ptr to Shot,
//   std::unordered_set<map::Shot *> local_keyfrms;

//   for (auto &keyfrm_weight : keyfrm_weights)
//   {
//     auto keyfrm = keyfrm_weight.first;
//     const auto weight = keyfrm_weight.second;
//     local_keyfrms.emplace(keyfrm);


//     // update the nearest keyframe
//     if (max_weight < weight)
//     {
//       max_weight = weight;
//     }
//   }
//   // add the second-order keyframes to the local landmarks
//   auto add_local_keyframe = [&](map::Shot *keyfrm) {
//     if (keyfrm == nullptr)
//     {
//       return false;
//     }
//     local_keyfrms.emplace(keyfrm);
//     return true;
//   };

//   const auto n_local_keyfrms = local_keyfrms.size();
//   for (auto iter = local_keyfrms.cbegin(), end = local_keyfrms.cend(); iter != end; ++iter)
//   {
//     if (max_num_local_keyfrms < n_local_keyfrms)
//     {
//       break;
//     }

//     auto keyfrm = *iter;

//     // covisibilities of the neighbor keyframe
//     const auto neighbors = keyfrm->slam_data_.graph_node_->get_top_n_covisibilities(10);
//     for (auto neighbor : neighbors)
//     {
//       if (add_local_keyframe(neighbor))
//       {
//         break;
//       }
//     }

//     // children of the spanning tree
//     const auto spanning_children = keyfrm->slam_data_.graph_node_->get_spanning_children();
//     for (auto child : spanning_children)
//     {
//       if (add_local_keyframe(child))
//       {
//         break;
//       }
//     }

//     // parent of the spanning tree
//     auto parent = keyfrm->slam_data_.graph_node_->get_spanning_parent();
//     add_local_keyframe(parent);
//   }
//   return std::vector<map::Shot *>(local_keyfrms.begin(), local_keyfrms.end());
// }

size_t
SlamUtilities::MatchShotToLocalMap(map::Shot &shot, const slam::GuidedMatcher& matcher)
{

  constexpr unsigned int max_num_local_keyfrms{60};
  
  //First create a set of landmarks that don't need matching, i.e. the already seen ones
  std::unordered_set<map::Landmark*> matched_lms;
  auto& lms = shot.GetLandmarks();
  // for (const auto& lm : shot.GetLandmarks())
  for (size_t i = 0; i < lms.size(); ++i)
  {
    auto* lm = lms[i];
    if (lm != nullptr)
    {
      matched_lms.insert(lm);
      lm->slam_data_.IncreaseNumObservable();
    }
  }
  std::vector<map::Landmark*> local_landmarks;
  // we get the local KFs!
  // e.g. KFs that see the same landmarks as the current frame
  // use a set to avoid duplicates
  std::unordered_set<map::Shot*> local_keyframes;
  //prevent adding the landmarks of the current shot
  local_keyframes.emplace(&shot); 
  // add the second-order keyframes to the local landmarks
  auto add_local_keyframe = [&](map::Shot *keyfrm) {
    if (keyfrm == nullptr)
    {
      return false;
    }
    const auto it = local_keyframes.emplace(keyfrm);
    if (!it.second) //try to add its landmarks
    {
      for (const auto& lm : keyfrm->GetLandmarks())
      {
        if (lm != nullptr && matched_lms.count(lm) == 0)
        {
          local_landmarks.push_back(lm);

          //Don't try again
          matched_lms.insert(lm);
        }
      }
    }
    return it.second;
  };

  const auto &landmarks = shot.GetLandmarks();
  const auto n_keypts = landmarks.size();
  for (unsigned int idx = 0; idx < n_keypts; ++idx)
  {
    const auto& lm = landmarks.at(idx);
    if (lm != nullptr)
    {
      for (const auto& obs : lm->GetObservations())
      {
        add_local_keyframe(obs.first);
      }
    }
  }

  if (local_keyframes.empty())
  {
    return 0;
  }

  // Try to insert the max_num number of keyframes
  for (auto iter = local_keyframes.cbegin(), end = local_keyframes.cend();
       iter != end && max_num_local_keyfrms >= local_keyframes.size(); ++iter)
  {
    auto keyfrm = *iter;
    // covisibilities of the neighbor keyframe
    const auto neighbors = keyfrm->slam_data_.graph_node_->get_top_n_covisibilities(10);
    for (auto neighbor : neighbors)
    {
      if (add_local_keyframe(neighbor))
      {
        break;
      }
    }
    // children of the spanning tree
    const auto spanning_children = keyfrm->slam_data_.graph_node_->get_spanning_children();
    for (auto child : spanning_children)
    {
      if (add_local_keyframe(child))
      {
        break;
      }
    }

    // parent of the spanning tree
    auto parent = keyfrm->slam_data_.graph_node_->get_spanning_parent();
    add_local_keyframe(parent);
  }

  constexpr float margin{5};
  constexpr float lowe_ratio{10};
  return matcher.AssignLandmarksToShot(shot, local_landmarks, margin,
                                       AlignedVector<Observation>(),GuidedMatcher::NO_ORIENTATION_CHECK,
                                       lowe_ratio);
}

std::unordered_map<map::ShotId, map::Shot*>
SlamUtilities::ComputeLocalKeyframes(map::Shot& shot)
{
  std::unordered_map<map::ShotId, map::Shot*> local_keyframes;
  local_keyframes[shot.id_] = &shot;

  const auto curr_covisibilities = shot.slam_data_.graph_node_->get_covisibilities();
  for (const auto& local_kf : curr_covisibilities)
  {
    if (local_kf != nullptr)
    {
      local_keyframes[local_kf->id_] = local_kf;
    }
  }
  return local_keyframes;
}

void
SlamUtilities::SetDescriptorFromObservations(map::Landmark& landmark)
{
  const auto &observations = landmark.GetObservations();
  if (!observations.empty())
  {
    AlignedVector<DescriptorType> descriptors;
    descriptors.reserve(observations.size());
    for (const auto &observation : observations)
    {
      auto shot = observation.first;
      const auto idx = observation.second;
      descriptors.push_back(shot->GetDescriptor(idx));
    }
    const auto median_idx = GuidedMatcher::ComputeMedianDescriptorIdx(descriptors);
    landmark.slam_data_.descriptor_ = descriptors[median_idx];
  }
}

void 
SlamUtilities::SetNormalAndDepthFromObservations(map::Landmark& landmark, const std::vector<float>& scale_factors)
{
  const auto &observations = landmark.GetObservations();
  if (observations.empty())
  {
    return;
  }
  // std::cout << "landmark: " << landmark.id_ << std::endl;
  Vec3d mean_normal = Vec3d::Zero();
  unsigned int num_observations = 0;
  for (const auto &observation : observations)
  {
    const auto shot = observation.first;
    const Vec3d cam_center = shot->GetPose().GetOrigin(); //.cast<float>();//get_cam_center();
    const Vec3d normal = landmark.GetGlobalPos() - cam_center;
    mean_normal = mean_normal + normal / normal.norm();
    ++num_observations;
    // std::cout << "shot: " << shot << ", " << shot->id_ << std::endl;
  }
  const auto dist = landmark.ComputeDistanceFromRefFrame();
  auto *ref_shot = landmark.GetRefShot();
  // std::cout << "ref_shot: "<< ref_shot << ", " << ref_shot->id_ << std::endl;
  const auto ref_obs_idx = observations.at(ref_shot);
  // std::cout << "res_obs_idx: " << ref_obs_idx << std::endl;
  const auto scale_level = ref_shot->slam_data_.undist_keypts_.at(ref_obs_idx).octave;
  // const auto scale_level = ref_shot_->undist_keypts_.at(observations_.at(ref_shot_)).octave;
  const auto scale_factor = scale_factors.at(scale_level);
  const auto num_scale_levels = scale_factors.size();

  landmark.slam_data_.max_valid_dist_ = dist * scale_factor;
  landmark.slam_data_.min_valid_dist_ = landmark.slam_data_.max_valid_dist_ / scale_factors.at(num_scale_levels - 1);
  landmark.slam_data_.mean_normal_ = mean_normal / num_observations;
}

std::pair<double, double>
SlamUtilities::ComputeMinMaxDepthInShot(const map::Shot& shot)
{

    double min_d{std::numeric_limits<double>::infinity()}, max_d{0};
    // const Eigen::Matrix4d T_cw = shot.GetWorldToCam();
    const auto& landmarks = shot.GetLandmarks();
    const auto& pose = shot.GetPose();
    const Mat3d R_cw = pose.RotationWorldToCamera();
    const Vec3d t_cw = pose.TranslationWorldToCamera();
    const Vec3d last_row = R_cw.row(2);
    const double t_z = t_cw[2];
    const auto n_landmarks{landmarks.size()};
    for (size_t idx = 0; idx < n_landmarks; ++idx)
    {
        auto* lm = landmarks[idx];
        if (lm != nullptr)
        {
          // const float d = (R_cw*lm->GetGlobalPos()+t_cw)[2];
          //should be equal
          const auto d = (last_row.dot(lm->GetGlobalPos())+t_z);

          min_d = std::min(d, min_d);
          max_d = std::max(d, max_d);
        }
    }
    return std::make_pair(min_d, max_d);
}


void 
SlamUtilities::FuseDuplicatedLandmarks(map::Shot& shot, const std::vector<map::Shot*>& fuse_shots, const slam::GuidedMatcher& matcher,
                                       const float margin,
                                       map::Map& slam_map)
{
  if (fuse_shots.empty())
  {
    return;
  }

  auto& landmarks = shot.GetLandmarks();

  for (const auto& fuse_shot: fuse_shots)
  {
    const auto n_fused = matcher.ReplaceDuplicatedLandmarks(*fuse_shot, landmarks, margin, slam_map);
  }
}

std::vector<map::Shot*>
SlamUtilities::GetSecondOrderCovisibilityForShot(const map::Shot& shot, const size_t first_order_thr, const size_t second_order_thr)
{
    const auto cur_covisibilities = shot.slam_data_.graph_node_->get_top_n_covisibilities(first_order_thr);
    std::set<map::Shot*, map::KeyCompare> fuse_tgt_keyfrms;

    for (const auto first_order_covis : cur_covisibilities) {
        if (fuse_tgt_keyfrms.count(first_order_covis) == 0)
        {
          fuse_tgt_keyfrms.insert(first_order_covis);

          // get the covisibilities of the covisibility of the current keyframe
          const auto ngh_covisibilities = first_order_covis->slam_data_.graph_node_->get_top_n_covisibilities(second_order_thr);
          for (const auto second_order_covis : ngh_covisibilities) 
          {
              // "the covisibilities of the covisibility" contains the current keyframe
              if (*second_order_covis != shot) {
                fuse_tgt_keyfrms.insert(second_order_covis);
              }
          }
        }
    }

    std::vector<map::Shot*> fuse_tgt_keyfrms_vec(fuse_tgt_keyfrms.cbegin(), fuse_tgt_keyfrms.cend());
    return fuse_tgt_keyfrms_vec;
}

void SlamUtilities::Triangulate(const TrackId track, const TracksManager& tracks_manager, map::Map& map, const double reproj_threshold, const double min_reproj_angle)
{
  MatX3d os, bs;
  std::vector<std::pair<map::Shot*, int>> shots;
  size_t added = 0;
  shots.clear();
  // triangulate
  for (const auto& shot_obs : tracks_manager.GetTrackObservations(track))
  {
    auto* shot = map.GetShot(shot_obs.first);
    if (shot != nullptr) 
    {
      const Vec3d bearing = shot->shot_camera_->Bearing(shot_obs.second.point);
      // //compare to compt bearing
      const auto feat_id = shot_obs.second.id;
      //TODO: buffer the rot, origin somewhere
      const auto& shot_pose = shot->GetPose();
      bs.conservativeResize(added + 1, Eigen::NoChange);
      os.conservativeResize(added + 1, Eigen::NoChange);
      bs.row(added) = shot_pose.RotationCameraToWorld() * bearing;
      os.row(added) = shot_pose.GetOrigin();
      shots.push_back(std::make_pair(shot, feat_id));
      ++added;
    }
  }
  const auto n_tracks = added;
  const std::vector<double> thresholds(n_tracks, reproj_threshold);

  // now triangulate_bearings_midpoint
  if (n_tracks >= 2)
  {
    const auto tri = geometry::TriangulateBearingsMidpointC(os, bs, thresholds, min_reproj_angle);
    const auto tri_status = tri.first;
    if (tri_status == geometry::TRIANGULATION_OK)
    {
      auto* lm = map.CreateLandmark(track, tri.second);

      for (const auto& shot_featid : shots)
      {
        map.AddObservation(shot_featid.first, lm, shot_featid.second);
      }
    }
  }
}

void SlamUtilities::TriangulateShotFeatures(const TracksManager& tracks_manager, map::Map& map, map::Shot* shot,
                       const double reproj_threshold, const double min_reproj_angle)
{
  const auto& tracks = tracks_manager.GetShotObservations(shot->id_);
  const auto& cam = shot->shot_camera_;
  for (const auto& track : tracks)
  {
    // if track not in reconstruction.points:
    if (!map.HasLandmark(track.first))
    {
      //triangulate!
      Triangulate(track.first, tracks_manager, map, reproj_threshold, min_reproj_angle);
    }
    // triangulate end 
  }
}

void SlamUtilities::Retriangulate(const TracksManager& tracks_manager, map::Map& map, const double reproj_threshold, const double min_reproj_angle,
                            const bool full_triangulation)
{
  if (!full_triangulation)
  {
    std::cout << "Only full triangulation implemented" << std::endl;
    exit(0);
  }

  map.ClearObservationsAndLandmarks();

  
  // check if the shot is in the reconstruction
  const auto& map_shots = map.GetAllShots();
  std::unordered_set<TrackId> tracks;
  // for image in reconstruction.shots.keys():
  for (const auto& id_shot : map_shots)
  {
    // const auto& shot = id_shot.second;
    const auto& shot_id = id_shot.first;
    // if image in all_shots_ids:
    // typically the number of shots in the map is far lower than in the tracks manager
    // TODO: think about a faster implementation!
    // if (all_shot_ids.count(shot.id_) > 0)
    
    if (tracks_manager.HasShotObservations(shot_id))
    {
      const auto& lm_id_obs = tracks_manager.GetShotObservations(shot_id);
      std::transform(lm_id_obs.cbegin(), lm_id_obs.cend(), std::inserter(tracks, tracks.begin()),
                [](const std::pair<TrackId, Observation>& p){ return p.first; });
    }
  }



  for (const auto& track : tracks)
  {
    if (full_triangulation)
      Triangulate(track, tracks_manager, map, reproj_threshold, min_reproj_angle);
    // else
      // TriangulateRobust
  }
}


} // namespace slam