
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

// Mat3f
// SlamUtilities::create_E_21(const Mat3f &rot_1w, const Vec3f &trans_1w,
//                            const Mat3f &rot_2w, const Vec3f &trans_2w)
// {
//   const Mat3f rot_21 = rot_2w * rot_1w.transpose();
//   const Vec3f trans_21 = -rot_21 * trans_1w + trans_2w;
//   const Mat3f trans_21_x = to_skew_symmetric_mat(trans_21);
//   return trans_21_x * rot_21;
// }

Mat3d
SlamUtilities::create_E_21(const Mat3d &rot_1w, const Vec3d &trans_1w,
                           const Mat3d &rot_2w, const Vec3d &trans_2w)
{
  const Mat3d rot_21 = rot_2w * rot_1w.transpose();
  const Vec3d trans_21 = -rot_21 * trans_1w + trans_2w;
  const Mat3d trans_21_x = to_skew_symmetric_mat(trans_21);
  return trans_21_x * rot_21;
}

bool SlamUtilities::check_epipolar_constraint(const Vec3f &bearing_1, const Vec3f &bearing_2,
                                              const Mat3f &E_12, const float bearing_1_scale_factor)
{
  // keyframe1上のtエピポーラ平面の法線ベクトル
  const Vec3f epiplane_in_1 = E_12 * bearing_2;

  // 法線ベクトルとbearingのなす角を求める
  const auto cos_residual = epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm();
  const auto residual_rad = M_PI / 2.0 - std::abs(std::acos(cos_residual));

  // inlierの閾値(=0.2deg)
  // (e.g. FOV=90deg,横900pixのカメラにおいて,0.2degは横方向の2pixに相当)
  // TODO: 閾値のパラメータ化
  constexpr double residual_deg_thr = 0.2;
  constexpr double residual_rad_thr = residual_deg_thr * M_PI / 180.0;

  // 特徴点スケールが大きいほど閾値を緩くする
  // TODO: thresholdの重み付けの検討
  return residual_rad < residual_rad_thr * bearing_1_scale_factor;
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
      // std::cout << "kpt: " << kpt.point[0] <<"," << kpt.point[1] << "," << kpt.scale << "," <<  kpt.angle << ", " << kpt.octave << std::endl;
    }
    return mat;
  }
  return MatXd();
}

std::vector<map::Landmark *>
SlamUtilities::update_local_landmarks(const std::vector<map::Shot *> &local_keyframes)
{
  // std::vector<map::Landmark*> local_landmarks;
  std::unordered_set<map::Landmark *> local_landmarks;
  for (auto keyframe : local_keyframes)
  {
    for (auto lm : keyframe->GetLandmarks())
    {
      if (lm != nullptr)
      {
        // do not add twice
        // if (lm->identifier_in_local_map_update_ == curr_frm_id) continue;
        // lm->identifier_in_local_map_update_ = curr_frm_id;
        local_landmarks.emplace(lm);
      }
    }
  }
  return std::vector<map::Landmark *>(local_landmarks.begin(), local_landmarks.end());
}

std::vector<map::Shot *>
SlamUtilities::update_local_keyframes(const map::Shot &curr_shot)
{
  constexpr unsigned int max_num_local_keyfrms{60};

  // count the number of sharing landmarks between the current frame and each of the neighbor keyframes
  // key: keyframe, value: number of sharing landmarks
  // std::unordered_map<KeyFrame*, unsigned int, KeyFrameCompare> keyfrm_weights;
  std::unordered_map<map::Shot *, unsigned int> keyfrm_weights;
  const auto &landmarks = curr_shot.GetLandmarks();
  const auto n_keypts = landmarks.size();
  for (unsigned int idx = 0; idx < n_keypts; ++idx)
  {
    auto lm = landmarks.at(idx);
    if (lm != nullptr)
    {
      // continue;
      // }
      // if (lm->will_be_erased()) {
      // std::cout << "lm->will_be_erased()" << std::endl;
      // exit(0);
      // kf.landmarks_.at(idx) = nullptr;
      //TODO: Write this maybe in a clean-up function!
      // frame.landmarks_.at(idx) = nullptr;
      // continue;
      // }

      const auto &observations = lm->GetObservations();
      for (const auto &obs : observations)
      {
        ++keyfrm_weights[obs.first];
      }
    }
  }

  if (keyfrm_weights.empty())
  {
    return std::vector<map::Shot *>();
  }

  // set the aforementioned keyframes as local keyframes
  // and find the nearest keyframe
  unsigned int max_weight = 0;
  // map::Shot* nearest_covisibility = nullptr;

  //ptr to Shot,
  // std::vector<map::Shot*> local_keyfrms;
  std::unordered_set<map::Shot *> local_keyfrms;
  // local_keyfrms.reserve(4 * keyfrm_weights.size());

  for (auto &keyfrm_weight : keyfrm_weights)
  {
    auto keyfrm = keyfrm_weight.first;
    const auto weight = keyfrm_weight.second;

    // if (keyfrm->will_be_erased()) {
    //     continue;
    // }

    // local_keyfrms.push_back(keyfrm);
    local_keyfrms.emplace(keyfrm);

    // avoid duplication
    // keyfrm->local_map_update_identifier = frame.frame_id;

    // update the nearest keyframe
    if (max_weight < weight)
    {
      max_weight = weight;
      // nearest_covisibility = keyfrm;
    }
  }
  std::cout << "local_keyfrms1: " << local_keyfrms.size() << std::endl;
  // add the second-order keyframes to the local landmarks
  auto add_local_keyframe = [&](map::Shot *keyfrm) {
    if (keyfrm == nullptr)
    {
      return false;
    }
    // if (keyfrm->will_be_erased()) {
    //     return false;
    // }
    // avoid duplication
    // if (keyfrm->local_map_update_identifier == frame.frame_id) {
    //     return false;
    // }
    // keyfrm->local_map_update_identifier = frame.frame_id;
    local_keyfrms.emplace(keyfrm);
    return true;
  };
  std::cout << "local_keyfrms2: " << local_keyfrms.size() << std::endl;
  const auto &n_local_keyfrms = local_keyfrms.size();
  for (auto iter = local_keyfrms.cbegin(), end = local_keyfrms.cend(); iter != end; ++iter)
  {
    if (max_num_local_keyfrms < n_local_keyfrms)
    {
      break;
    }

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
  std::cout << "local_keyfrms: " << local_keyfrms.size() << std::endl;
  return std::vector<map::Shot *>(local_keyfrms.begin(), local_keyfrms.end());
}

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
      // std::cout << "Already matched: " << lm->id_ << ", " << lm << ", "<< i << std::endl;
    }
  }
  // std::cout << "Created matched lms!" << std::endl;
  std::vector<map::Landmark*> local_landmarks;
  // std::vector<std::pair<map::Landmark *, Vec3d>,
  //             Eigen::aligned_allocator<Vec3d>>
  //     local_landmarks; //Vector3f stores the reprojected point and its pred. scale
  // we get the local KFs!
  // e.g. KFs that see the same landmarks as the current frame
  // use a set to avoid duplicates
  std::unordered_set<map::Shot*> local_keyframes;
  //prevent adding the landmarks of the current shot
  local_keyframes.emplace(&shot); 
  // Eigen::Vector2d reproj;
  // size_t scale_level;
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
          // //try to reproject
          // if (matcher.IsObservable(lm, *keyfrm, 0.5, reproj, scale_level))
          // {
          //   lm->slam_data_.IncreaseNumObservable();
          //   // TODO: Somehow we have to do the increase num observable in the matching!
          //   local_landmarks.push_back(lm);
          //   // local_landmarks.emplace_back(
          //   //   std::make_pair(lm, Vec3d(reproj[0], reproj[1], scale_level)));
          // }
          // std::cout << "Emplacing: " << lm->id_ << ", " << lm << std::endl;

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
      // std::cout << "Processing: " << lm->id_ << std::endl;
      for (const auto& obs : lm->GetObservations())
      {
        // local_keyframes.insert(obs.first);
        // std::cout << "Adding " << obs.first << std::endl;
        add_local_keyframe(obs.first);
      }
    }
  }
  // std::cout << "Created local lms and kfs!" << std::endl;

  if (local_keyframes.empty())
  {
    return 0;
  }


  // // add the second-order keyframes to the local landmarks
  // auto add_local_keyframe = [&](map::Shot *keyfrm) {
  //   if (keyfrm == nullptr)
  //   {
  //     return false;
  //   }
  //   const auto it = local_keyframes.emplace(keyfrm);
  //   return it.second;
  // };

  // Try to insert the max_num number of keyframes
  for (auto iter = local_keyframes.cbegin(), end = local_keyframes.cend();
       iter != end && max_num_local_keyfrms >= local_keyframes.size(); ++iter)
  {
    // std::cout << "getting desc!" << std::endl;
    auto keyfrm = *iter;
    // std::cout << "local_keyframes.size()" << local_keyframes.size() << std::endl;
    // covisibilities of the neighbor keyframe
    const auto neighbors = keyfrm->slam_data_.graph_node_->get_top_n_covisibilities(10);
    for (auto neighbor : neighbors)
    {
      if (add_local_keyframe(neighbor))
      {
        break;
      }
    }
    // std::cout << "add neighbor local_keyframes.size()" << local_keyframes.size() << std::endl;
    // children of the spanning tree
    const auto spanning_children = keyfrm->slam_data_.graph_node_->get_spanning_children();
    for (auto child : spanning_children)
    {
      if (add_local_keyframe(child))
      {
        break;
      }
    }
    // std::cout << "add child local_keyframes.size()" << local_keyframes.size() << std::endl;

    // parent of the spanning tree
    auto parent = keyfrm->slam_data_.graph_node_->get_spanning_parent();
    add_local_keyframe(parent);
    // std::cout << "add parent local_keyframes.size()" << local_keyframes.size() << std::endl;
  }
  // std::cout << "local_keyfrms: " << local_keyframes.size() << std::endl;

  // std::unordered_set<map::Landmark*> test_lm;
  // for (auto lm : local_landmarks)
  // {
  //   test_lm.insert(lm);
  // }
  // if (test_lm.size() != local_landmarks.size())
  // {
  //   std::cout << "test_lm.size() != local_landmarks.size()" << std::endl;
  //   exit(0);
  // }
  //     std::cout << "test_lm.size() == local_landmarks.size()" << std::endl;

  // return local_keyframes.size();
  // we get the local landmarks
  // Landmarks seen by the local KFs
  // std::unordered_set<map::Landmark*> local_landmarks;
  // for (auto keyframe : local_keyframes)
  // {
  //     for (auto lm : keyframe->GetLandmarks())
  //     {
  //         if (lm != nullptr)
  //         {
  //           local_landmarks.emplace(lm);
  //         }
  //     }
  // }

  // convert landmarks seen in current shot to unordered_set

  //make new vector
  // only add:
  // landmarks not in the set
  // and visible in the current frame

  //Assign landmarks to current frame
  // return local_keyframes.size();
  
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

// void 
// SlamUtilities::SetDescriptorFromObservations(map::Landmark& landmark)
// {
//   const auto& observations = landmark.GetObservations();
//   if (observations.empty()) {
//       return;
//   }
//   std::vector<cv::Mat> descriptors;
//   descriptors.reserve(observations.size());
//   for (const auto& observation : observations) {
//       auto shot = observation.first;
//       const auto idx = observation.second;
//       descriptors.push_back(shot->GetDescriptor(idx));
//   }
//   const auto median_idx = GuidedMatcher::ComputeMedianDescriptorIdx(descriptors);
//   landmark.slam_data_.descriptor_ = descriptors[median_idx].clone();
// }

void
SlamUtilities::SetDescriptorFromObservations(map::Landmark& landmark)
{
  const auto &observations = landmark.GetObservations();
  if (observations.empty())
  {
    return;
  }
  AlignedVector<DescriptorType> descriptors;
  descriptors.reserve(observations.size());
  for (const auto &observation : observations)
  {
    auto shot = observation.first;
    const auto idx = observation.second;
    descriptors.push_back(shot->GetDescriptor(idx));
  }
  const auto median_idx = GuidedMatcher::ComputeMedianDescriptorIdx(descriptors);
  landmark.slam_data_.descriptor_ = descriptors[median_idx]; //.clone();
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
    std::cout << "Fused " << n_fused << " landmarks for " << fuse_shot->id_ << ", " << fuse_shot->unique_id_ << std::endl;
  }
}

std::vector<map::Shot*>
// std::set<map::Shot*, map::KeyCompare>
SlamUtilities::GetSecondOrderCovisibilityForShot(const map::Shot& shot, const size_t first_order_thr, const size_t second_order_thr)
{
    const auto cur_covisibilities = shot.slam_data_.graph_node_->get_top_n_covisibilities(first_order_thr);
    std::cout << "cur_covisibilities: " << cur_covisibilities.size() << std::endl;
    // std::unordered_set<KeyFrame*> fuse_tgt_keyfrms;
    std::set<map::Shot*, map::KeyCompare> fuse_tgt_keyfrms;
    // fuse_tgt_keyfrms.reserve(cur_covisibilities.size() * 2);

    for (const auto first_order_covis : cur_covisibilities) {
        // if (first_order_covis->will_be_erased()) {
        //     continue;
        // }

        // check if the keyframe is aleady inserted
        // if (static_cast<bool>(fuse_tgt_keyfrms.count(first_order_covis))) {
        //     continue;
        // }

        if (fuse_tgt_keyfrms.count(first_order_covis) == 0)
        {
          fuse_tgt_keyfrms.insert(first_order_covis);
          std::cout << "fuse_tgt_keyfrms.insert(first_order_covis): " << first_order_covis->id_ << std::endl;

          // get the covisibilities of the covisibility of the current keyframe
          const auto ngh_covisibilities = first_order_covis->slam_data_.graph_node_->get_top_n_covisibilities(second_order_thr);
          for (const auto second_order_covis : ngh_covisibilities) 
          {
              // if (second_order_covis->will_be_erased()) {
              //     continue;
              // }
              // "the covisibilities of the covisibility" contains the current keyframe
              if (*second_order_covis != shot) {
                fuse_tgt_keyfrms.insert(second_order_covis);
                std::cout << "fuse_tgt_keyfrms.insert(second_order_covis): " << second_order_covis->id_ << std::endl;
              }
          }
        }
    }

    for (const auto& frm: fuse_tgt_keyfrms)
    {
        std::cout << "frm: " << frm->id_ << "/" << frm->unique_id_ << std::endl;
    }
    // return fuse_tgt_keyfrms;
    // TODO: this copy is unnecessary and only used to keep the order
    std::vector<map::Shot*> fuse_tgt_keyfrms_vec(fuse_tgt_keyfrms.cbegin(), fuse_tgt_keyfrms.cend());
    return fuse_tgt_keyfrms_vec;
}

void SlamUtilities::Triangulate(const TrackId track, const TracksManager& tracks_manager, map::Map& map, const double reproj_threshold, const double min_reproj_angle)
{
  // const auto& cam = shot->shot_camera_.camera_model_;
  // Eigen::Matrix<double, Eigen::Dynamic, 3> os;
  // Eigen::Matrix<double, Eigen::Dynamic, 3> bs;
  MatX3d os, bs;
  std::vector<std::pair<map::Shot*, int>> shots;
  size_t added = 0;
  shots.clear();
  // triangulate
  //for shot_id, obs in self.tracks_manager.get_track_observations(track).items():
  for (const auto& shot_obs : tracks_manager.GetTrackObservations(track))
  {
    auto* shot = map.GetShot(shot_obs.first);
    if (shot != nullptr) 
    {
      const Vec3d bearing = shot->shot_camera_->Bearing(shot_obs.second.point);
      // //compare to compt bearing
      const auto feat_id = shot_obs.second.id;
      // Vec3d bearing = shot->slam_data_.bearings_[feat_id];
      // std::cout << "bearing: " << bearing << " b2: " << b2 << " obs: " << shot_obs.second.point << std::endl;
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
  // std::cout << lm_id << ": " << added << ", "<< bs << ", " << os << 
  //             "reproj_threshold" << reproj_threshold << std::endl;

  // now triangulate_bearings_midpoint
  if (n_tracks >= 2)
  {
    // std::cout << "min_reproj_angle: " << min_reproj_angle << std::endl;
    const auto tri = geometry::TriangulateBearingsMidpointC(os, bs, thresholds, min_reproj_angle);
    const auto tri_status = tri.first;
    // std::cout << "tri_status: " << tri_status << "," << tri.second << std::endl;
    if (tri_status == geometry::TRIANGULATION_OK)
    {
      // auto* lm = map.CreateLandmark(std::stoi(track), tri.second);
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
  // AlignedVector<Vec3d> bs;
  const auto& cam = shot->shot_camera_;
  // std::map<std::string, std::pair<Vec3d, Mat3d>> buffer;
  // AlignedVector<Vec3d> os;
  // const auto n_shots = map.NumberOfShots();
  // Eigen::Matrix<double, Eigen::Dynamic, 3> os(2, 3);
  // Eigen::Matrix<double, Eigen::Dynamic, 3> bs(2, 3);
  // std::vector<std::pair<map::Shot*, int>> shots;
  // for track in tracks_manager.get_shot_observations(shot_id):
  for (const auto& track : tracks)
  {
    // if track not in reconstruction.points:
    // const auto lm_id = track.first; //std::stoi(track.first);
    if (!map.HasLandmark(track.first))
    {
      //triangulate!
      Triangulate(track.first, tracks_manager, map, reproj_threshold, min_reproj_angle);
      // size_t added = 0;
      // shots.clear();
      // // triangulate
      // //for shot_id, obs in self.tracks_manager.get_track_observations(track).items():
      // for (const auto& shot_obs : tracks_manager.GetTrackObservations(track.first))
      // {
      //   auto* shot = map.GetShot(shot_obs.first);
      //   if (shot != nullptr) 
      //   {
      //     Vec3d bearing = cam.PixelBearing(shot_obs.second.point);
      //     // //compare to compt bearing
      //     const auto feat_id = shot_obs.second.id;
      //     // Vec3d bearing = shot->slam_data_.bearings_[feat_id];
      //     // std::cout << "bearing: " << bearing << " b2: " << b2 << " obs: " << shot_obs.second.point << std::endl;
      //     //TODO: buffer the rot, origin somewhere
      //     const auto& shot_pose = shot->GetPose();
      //     bs.row(added) = shot_pose.RotationCameraToWorld()*bearing;
      //     os.row(added) = shot_pose.GetOrigin();
      //     shots.push_back(std::make_pair(shot, feat_id));
      //     ++added;
      //   }
      // }
      // const auto n_tracks = added;
      // const std::vector<double> thresholds(n_tracks, reproj_threshold);
      // // std::cout << lm_id << ": " << added << ", "<< bs << ", " << os << 
      // //             "reproj_threshold" << reproj_threshold << std::endl;

      // // now triangulate_bearings_midpoint
      // if (n_tracks >= 2)
      // {
      //   // std::cout << "min_reproj_angle: " << min_reproj_angle << std::endl;
      //   const auto tri = geometry::TriangulateBearingsMidpointC(os, bs, thresholds, min_reproj_angle);
      //   const auto tri_status = tri.first;
      //   // std::cout << "tri_status: " << tri_status << "," << tri.second << std::endl;
      //   if (tri_status == geometry::TRIANGULATION_OK)
      //   {
      //     auto* lm = map.CreateLandmark(lm_id, tri.second);
      //     for (const auto& shot_featid : shots)
      //     {
      //       map.AddObservation(shot_featid.first, lm, shot_featid.second);
      //     }
      //   }
      // }
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

  // const auto& all_shot_ids = tracks_manager.GetShotIds();
  
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