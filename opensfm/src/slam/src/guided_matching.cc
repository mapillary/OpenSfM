#include <slam/guided_matching.h>
#include <slam/third_party/orb_extractor/util/angle_checker.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <unordered_set>
#include <geometry/camera.h>
#include <slam/slam_utilities.h>
#include <unordered_set>
namespace slam
{

GridParameters::GridParameters(unsigned int grid_cols, unsigned int grid_rows,
                               float img_min_width, float img_min_height,
                               float img_max_width, float img_max_height,
                               float inv_cell_width, float inv_cell_height) : 
                               grid_cols_(grid_cols), grid_rows_(grid_rows),
                               img_min_width_(img_min_width), img_min_height_(img_min_height),
                               img_max_width_(img_max_width), img_max_height_(img_max_height),
                               inv_cell_width_(inv_cell_width), inv_cell_height_(inv_cell_height)
{
}


 constexpr unsigned int GuidedMatcher::HAMMING_DIST_THR_LOW;
 constexpr unsigned int GuidedMatcher::HAMMING_DIST_THR_HIGH;
 constexpr unsigned int GuidedMatcher::MAX_HAMMING_DIST;
 constexpr float GuidedMatcher::NO_LOWE_TEST;
 constexpr bool GuidedMatcher::NO_ORIENTATION_CHECK;
 constexpr size_t GuidedMatcher::NO_MATCH;

size_t
GuidedMatcher::ComputeMedianDescriptorIdx(const AlignedVector<DescriptorType> &descriptors)
{
  // First, calculate the distance between features in all combinations
  const auto num_descs = descriptors.size();
  std::vector<std::vector<unsigned int>> hamm_dists(num_descs, std::vector<unsigned int>(num_descs));
  for (unsigned int i = 0; i < num_descs; ++i)
  {
    hamm_dists.at(i).at(i) = 0;
    for (unsigned int j = i + 1; j < num_descs; ++j)
    {
      const auto dist = compute_descriptor_distance_32(descriptors.at(i), descriptors.at(j));
      hamm_dists.at(i).at(j) = dist;
      hamm_dists.at(j).at(i) = dist;
    }
  }

  // Find the closest to the median
  unsigned int best_median_dist = MAX_HAMMING_DIST;
  unsigned int best_idx = 0;
  for (unsigned idx = 0; idx < num_descs; ++idx)
  {
    std::vector<unsigned int> partial_hamm_dists(hamm_dists.at(idx).begin(), hamm_dists.at(idx).begin() + num_descs);
    std::sort(partial_hamm_dists.begin(), partial_hamm_dists.end());
    const auto median_dist = partial_hamm_dists.at(static_cast<unsigned int>(0.5 * (num_descs - 1)));
    if (median_dist < best_median_dist)
    {
      best_median_dist = median_dist;
      best_idx = idx;
    }
  }
  return best_idx;
}


size_t 
GuidedMatcher::FindBestMatchForLandmark(const map::Landmark *const lm, map::Shot& curr_shot,
                                        const float reproj_x, const float reproj_y,
                                        const int scale_level, const float margin,
                                        const float lowe_ratio) const //lowe ratio == 0.0 if no test
{
  // std::cout << "margin * scale_factors_.at(scale_level): " << margin * scale_factors_.at(scale_level)
  //                 << ", " << margin << ", "<< scale_level << std::endl;
  // std::cout << "curr_shot.slam_data_.undist_keypts_: " << curr_shot.slam_data_.undist_keypts_.size()
  //           << std::endl;
  const auto indices = GetKeypointsInCell(curr_shot.slam_data_.undist_keypts_,
                                          curr_shot.slam_data_.keypt_indices_in_cells_, 
                                          reproj_x, reproj_y,
                                          scale_factors_.at(scale_level)*margin ,
                                          scale_level - 1, scale_level + 1);
                                        
  if (indices.empty())
  {
    // std::cout << "indices.empty(), no match" << std::endl;
    return NO_MATCH;
  }

  const auto lm_desc = lm->slam_data_.descriptor_;
  unsigned int best_hamm_dist = MAX_HAMMING_DIST;
  int best_scale_level = -1;
  int best_idx = -1;
  //for lowe test
  unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;
  int second_best_scale_level = -1;
  const auto& undist_kpts = curr_shot.slam_data_.undist_keypts_;

  for (const auto match_idx : indices)
  {
    const auto* curr_lm = curr_shot.GetLandmark(match_idx);
    //prevent adding to already set landmarks
    if (curr_lm == nullptr || (curr_lm != nullptr && !curr_lm->HasObservations()))
    {
      const auto& desc = curr_shot.GetDescriptor(match_idx);
      const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);
      // if (match_idx == 3140)
      // {
      //   std::cout << "matching: " << match_idx << ",  hamm: " << hamm_dist << std::endl;
      // }
      if (hamm_dist < best_hamm_dist) {
        second_best_hamm_dist = best_hamm_dist;
        best_hamm_dist = hamm_dist;
        second_best_scale_level = best_scale_level;
        best_scale_level = undist_kpts.at(match_idx).octave;
        best_idx = match_idx;
      }
      else 
        if (hamm_dist < second_best_hamm_dist) {
          second_best_scale_level = undist_kpts.at(match_idx).octave;
          second_best_hamm_dist = hamm_dist;
        }
    }
  }
  if (best_hamm_dist <= HAMMING_DIST_THR_HIGH)
  {
    //perform Lowe ratio test
    if (lowe_ratio > 0.0f && best_scale_level == second_best_scale_level && best_hamm_dist > lowe_ratio * second_best_hamm_dist) 
    {
      std::cout <<"Discard (lowe_ratio > 0.0f && best_scale_level == second_best_scale_level && best_hamm_dist > lowe_ratio * second_best_hamm_dist)" << std::endl;
      return NO_MATCH;
    }
    return best_idx;
  }
  // std::cout << "Discard HAMMING_DIST_THR_HIGH" << std::endl;
  return NO_MATCH;
}

MatchIndices
GuidedMatcher::MatchKptsToKpts(const AlignedVector<Observation>& undist_keypts_1, const DescriptorMatrix& descriptors_1,
                               const AlignedVector<Observation>& undist_keypts_2, const DescriptorMatrix& descriptors_2,
                               const CellIndices& keypts_indices_in_cells_2,
                               const Eigen::MatrixX2f& prevMatched, const size_t margin) const
{
  MatchIndices match_indices; // Index in 1, Index in 2
  if (undist_keypts_1.empty() || undist_keypts_2.empty() || keypts_indices_in_cells_2.empty())
  {
    return match_indices;
  }
  constexpr auto check_orientation_{true};
  constexpr float lowe_ratio_{0.9};
  const size_t num_pts_1 = undist_keypts_1.size();
  const size_t num_pts_2 = undist_keypts_2.size();

  std::vector<unsigned int> matched_dists_in_frm_2(num_pts_2, MAX_HAMMING_DIST);
  std::vector<int> matched_indices_1_in_frm_2(num_pts_2, -1);
  std::vector<int> matched_indices_2_in_frm_1 = std::vector<int>(num_pts_1, -1);
  size_t num_matches = 0; // Todo: should be the same as matches.size()
  openvslam::match::angle_checker<int> angle_checker;
  for (size_t idx_1 = 0; idx_1 < num_pts_1; ++idx_1)
  {
    // f1 = x, y, size, angle, octave
    const auto &u_kpt_1 = undist_keypts_1.at(idx_1);
    const Eigen::Vector2f pt2D = prevMatched.row(idx_1);
    const auto scale_1 = u_kpt_1.octave;
    if (scale_1 < 0)
      continue;
    const auto indices = GetKeypointsInCell(undist_keypts_2, keypts_indices_in_cells_2,
                                            pt2D[0], pt2D[1], margin, scale_1, scale_1);
    if (indices.empty())
      continue; // No valid match

    // Read the descriptor
    const auto &d1 = descriptors_1.row(idx_1);
    auto best_hamm_dist = MAX_HAMMING_DIST;
    auto second_best_hamm_dist = MAX_HAMMING_DIST;
    int best_idx_2 = -1;
    for (const auto idx_2 : indices)
    {
      const auto &d2 = descriptors_2.row(idx_2);
      const auto hamm_dist = compute_descriptor_distance_32(d1, d2);
      // through if the point already matched is closer
      if (matched_dists_in_frm_2.at(idx_2) <= hamm_dist)
      {
        continue;
      }
      if (hamm_dist < best_hamm_dist)
      {
        second_best_hamm_dist = best_hamm_dist;
        best_hamm_dist = hamm_dist;
        best_idx_2 = idx_2;
      }
      else if (hamm_dist < second_best_hamm_dist)
      {
        second_best_hamm_dist = hamm_dist;
      }
    }

    if (HAMMING_DIST_THR_LOW < best_hamm_dist)
    {
      continue;
    }

    // ratio test
    if (second_best_hamm_dist * lowe_ratio_ < static_cast<float>(best_hamm_dist))
    {
      continue;
    }

    const auto prev_idx_1 = matched_indices_1_in_frm_2.at(best_idx_2);
    if (0 <= prev_idx_1)
    {
      matched_indices_2_in_frm_1.at(prev_idx_1) = -1;
      --num_matches;
    }

    // 互いの対応情報を記録する
    matched_indices_2_in_frm_1.at(idx_1) = best_idx_2;
    matched_indices_1_in_frm_2.at(best_idx_2) = idx_1;
    matched_dists_in_frm_2.at(best_idx_2) = best_hamm_dist;
    ++num_matches;

    if (check_orientation_)
    {
      const auto delta_angle = undist_keypts_1.at(idx_1).angle - undist_keypts_2.at(best_idx_2).angle;
      angle_checker.append_delta_angle(delta_angle, idx_1);
    }
  }

  if (check_orientation_)
  {
    const auto invalid_matches = angle_checker.get_invalid_matches();
    for (const auto invalid_idx_1 : invalid_matches)
    {
      if (0 <= matched_indices_2_in_frm_1.at(invalid_idx_1))
      {
        matched_indices_2_in_frm_1.at(invalid_idx_1) = -1;
        --num_matches;
      }
    }
  }

  match_indices.reserve(num_pts_1);
  for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_frm_1.size(); ++idx_1)
  {
    const auto idx_2 = matched_indices_2_in_frm_1.at(idx_1);
    if (idx_2 >= 0)
    {
      match_indices.emplace_back(std::make_pair(idx_1, idx_2));
    }
  }
  return match_indices;
}

void GuidedMatcher::DistributeUndistKeyptsToGrid(const AlignedVector<Observation>& undist_keypts, CellIndices& keypt_indices_in_cells) const

{
  const size_t num_pts = undist_keypts.size();
  const size_t num_to_reserve = 0.5 * num_pts / (grid_params_.grid_cols_ * grid_params_.grid_rows_);
  keypt_indices_in_cells.resize(grid_params_.grid_cols_);
  for (auto &keypt_indices_in_row : keypt_indices_in_cells)
  {
    keypt_indices_in_row.resize(grid_params_.grid_rows_);
    for (auto &keypt_indices_in_cell : keypt_indices_in_row)
    {
      keypt_indices_in_cell.reserve(num_to_reserve);
    }
  }
  for (size_t idx = 0; idx < num_pts; ++idx)
  {
    const auto &keypt = undist_keypts.at(idx);
    const int cell_idx_x = std::round((keypt.point[0] - grid_params_.img_min_width_) * grid_params_.inv_cell_width_);
    const int cell_idx_y = std::round((keypt.point[1] - grid_params_.img_min_height_) * grid_params_.inv_cell_height_);
    if ((0 <= cell_idx_x && cell_idx_x < static_cast<int>(grid_params_.grid_cols_) && 0 <= cell_idx_y && cell_idx_y < static_cast<int>(grid_params_.grid_rows_)))
    {
      keypt_indices_in_cells.at(cell_idx_x).at(cell_idx_y).push_back(idx);
    }
  }
}

std::vector<size_t>
GuidedMatcher::GetKeypointsInCell(const AlignedVector<Observation>& undist_keypts,
                                  const CellIndices& keypt_indices_in_cells,
                                  const float ref_x, const float ref_y, const float margin,
                                  const int min_level, const int max_level) const
{
  std::vector<size_t> indices;
  indices.reserve(undist_keypts.size());
  const int min_cell_idx_x = std::max(0, cvFloor((ref_x - grid_params_.img_min_width_ - margin) * grid_params_.inv_cell_width_));

  if (static_cast<int>(grid_params_.grid_cols_) <= min_cell_idx_x)
  {
    return indices;
  }

  const int max_cell_idx_x = std::min(static_cast<int>(grid_params_.grid_cols_ - 1), cvCeil((ref_x - grid_params_.img_min_width_ + margin) * grid_params_.inv_cell_width_));
  if (max_cell_idx_x < 0)
  {
    return indices;
  }

  const int min_cell_idx_y = std::max(0, cvFloor((ref_y - grid_params_.img_min_height_ - margin) * grid_params_.inv_cell_height_));
  if (static_cast<int>(grid_params_.grid_rows_) <= min_cell_idx_y)
  {
    return indices;
  }

  const int max_cell_idx_y = std::min(static_cast<int>(grid_params_.grid_rows_ - 1), cvCeil((ref_y - grid_params_.img_min_height_ + margin) * grid_params_.inv_cell_height_));
  if (max_cell_idx_y < 0)
  {
    return indices;
  }

  const bool check_level = (0 < min_level) || (0 <= max_level);
  for (int cell_idx_x = min_cell_idx_x; cell_idx_x <= max_cell_idx_x; ++cell_idx_x)
  {
    for (int cell_idx_y = min_cell_idx_y; cell_idx_y <= max_cell_idx_y; ++cell_idx_y)
    {
      const auto &keypt_indices_in_cell = keypt_indices_in_cells.at(cell_idx_x).at(cell_idx_y);

      if (keypt_indices_in_cell.empty())
      {
        continue;
      }

      for (unsigned int idx : keypt_indices_in_cell)
      {
        const auto &keypt = undist_keypts[idx];
        if (check_level)
        {
          if (keypt.octave < min_level || (0 <= max_level && max_level < keypt.octave))
          {
            continue;
          }
        }
        const float dist_x = keypt.point[0] - ref_x;
        const float dist_y = keypt.point[1] - ref_y;
        if (std::abs(dist_x) < margin && std::abs(dist_y) < margin)
        {
          indices.push_back(idx);
          if (idx >= undist_keypts.size())
          {
            std::runtime_error("keypts idx error!"+std::to_string(idx));
            // std::cout << "keypts idx error!" << idx << std::endl;
            // exit(0);
          }
        }
      }
    }
  }
  return indices;
}

size_t
GuidedMatcher::AssignLandmarksToShot(map::Shot& shot, const std::vector<map::Landmark*>& landmarks, const float margin,
                                     const AlignedVector<Observation>& other_undist_kpts, bool check_orientation, const float lowe_ratio) const
{
  size_t num_matches{0};
  std::unique_ptr<openvslam::match::angle_checker<int>> angle_checker;
  if (landmarks.empty())
  {
    return 0;
  }
  if (other_undist_kpts.empty())
  {
    check_orientation = false;
  }
  if (check_orientation)    
  {
    angle_checker = std::unique_ptr<openvslam::match::angle_checker<int>>(new openvslam::match::angle_checker<int>());
  }
  
  const auto &cam_pose = shot.GetPose();
  const Mat3d rot_cw = cam_pose.RotationWorldToCamera();
  const Vec3d trans_cw = cam_pose.TranslationWorldToCamera();
  const Vec3d origin = cam_pose.GetOrigin();
  const auto& cam = shot.shot_camera_;
  const auto n_landmarks{landmarks.size()};
  Vec2d pt2D; // stores the reprojected position
  const auto& undist_kpts = shot.slam_data_.undist_keypts_;
  size_t n_scales = 0;
  size_t num_perfect_matches = 0;
  size_t discard_grid = 0, total = 0, no_match_disc = 0;
  // if (landmarks.size() != undist_kpts.size())
  // {
  //   std::runtime_error("landmarks.size() != undist_kpts.size()");
  // }
  // std::cout << "n_landmarks: " << n_landmarks << std::endl;
  for (size_t idx = 0; idx < n_landmarks; ++idx)
  {
    // std::cout << "idx: " << idx << std::endl;
    auto lm = landmarks[idx];
    if (lm != nullptr)
    {
      // Reproject
      const Vec3d& global_pos = lm->GetGlobalPos();
      const Vec2d pt2D = shot.ProjectInImageCoordinates(global_pos);
      // std::cout << "idx: " << idx << " pt2D: " << pt2D.transpose() << std::endl;
      ++total;
      // if (cam->CheckWithinBoundaries(pt2D))
      {
        //check if it is within our grid
        if (grid_params_.in_grid(pt2D[0], pt2D[1]))
        {
          auto& lm_data = lm->slam_data_;
          int scale_lvl = -1;
          if (!other_undist_kpts.empty()) //take the scale level from the keypts
          {
            scale_lvl = other_undist_kpts.at(idx).octave;
          }
          else // or predict
          {
            constexpr auto ray_cos_thr{0.5};
            const Vec3d cam_to_lm_vec = global_pos - origin;
            const auto cam_to_lm_dist = cam_to_lm_vec.norm();
            // check if inside orb sale
            if (lm_data.GetMinValidDistance()  <= cam_to_lm_dist && 
                lm_data.GetMaxValidDistance() >= cam_to_lm_dist)
            {
              const Vec3d obs_mean_normal = lm_data.mean_normal_;
              const auto ray_cos = cam_to_lm_vec.dot(obs_mean_normal) / cam_to_lm_dist;
              if (ray_cos > ray_cos_thr)
              {
                scale_lvl = PredScaleLevel(lm_data.GetMaxValidDistance(), cam_to_lm_dist);
              }
            }
          }
          if (scale_lvl >= 0)
          {
            lm_data.IncreaseNumObservable();
            const auto best_idx = FindBestMatchForLandmark(lm, shot, float(pt2D[0]), float(pt2D[1]), scale_lvl, margin, lowe_ratio);
            // if (best_idx == 3140)
            // {
            //   std::cout << "pt2D: " << pt2D.transpose() << ", " << other_undist_kpts[best_idx].point.transpose() << std::endl;
            // }
            if (best_idx != NO_MATCH)
            {
              if (check_orientation)
              {
                // std::cout << "best_idx: " << best_idx << "/" << idx << std::endl;
                const auto delta_angle = other_undist_kpts.at(idx).angle - undist_kpts.at(best_idx).angle;
                angle_checker->append_delta_angle(delta_angle, best_idx);
                // std::cout << idx << "<->" << best_idx <<
                //          "delta_angle: " << delta_angle << " langle: " << other_undist_kpts.at(idx).angle
                //       << " rangle: " << undist_kpts.at(best_idx).angle << 
                //       std::endl;

                // const DescriptorType d1 = shot.GetDescriptor(best_idx);
                // const DescriptorType d2 = lm->slam_data_.descriptor_;
                // const DescriptorType d3 = shot.GetDescriptor(idx);
                // std::cout << "best_idx<->desc: " << GuidedMatcher::compute_descriptor_distance_32(d1,d2)
                //           << "\nbest_idx<->idx: " << GuidedMatcher::compute_descriptor_distance_32(d1,d3)
                //           << "\ndesc<->idx:" << GuidedMatcher::compute_descriptor_distance_32(d2,d3) << std::endl;
              }
              if (idx == best_idx) {
                num_perfect_matches++;
              }
              num_matches++;
              shot.AddLandmarkObservation(lm, best_idx);
            }
            // else
            // {
            //   std::cout << "Couldn't match " << idx << std::endl; 
            //   ++no_match_disc;
            // }
          }
          // else
          // {
          //   std::cout << "Couldn't match " << idx << "because of scale" << std::endl; 
          //   ++n_scales;
          // }
        }
        // else
        // {
        //   std::cout << "Couldn't match " << idx << "because of grid" << std::endl; 
        //   ++discard_grid;
        // }
      }
    }
  }
  // std::cout << "discarded because of grid: " << discard_grid << "/" << n_scales << "/" << no_match_disc << "/" << total << std::endl;
  std::cout << "Number of matches before angle: " << num_matches << std::endl;
  if (check_orientation) 
  {
    const auto invalid_matches = angle_checker->get_invalid_matches();
    for (const auto invalid_idx : invalid_matches) {
        shot.RemoveLandmarkObservation(invalid_idx);
        --num_matches;
        // std::cout << "Remove match: " << invalid_idx << " because of angle!" << std::endl;
    }
  }
  std::cout << "Number of matches after angle: " << num_matches << std::endl;
  std::cout << "Number of perfect matches" << num_perfect_matches << std::endl;
  // exit(0);
  return num_matches;
}



MatchIndices
GuidedMatcher::MatchingForTriangulationEpipolar(const map::Shot& kf1, const map::Shot& kf2, const Mat3d& E_12, const float min_depth, const float max_depth, const bool traverse_with_depth, const float margin) const
{
  MatchIndices matches;
  // Typically, kf1 = current frame and kf2 = an older frame!
  // We assume that the old frame
  // get the already matched landmarks
  const auto& lms1 = kf1.GetLandmarks();
  const auto& lms2 = kf2.GetLandmarks();
  const auto& kf1_pose = kf1.GetPose();
  const auto& kf2_pose = kf2.GetPose();
  const Vec3d cam_center_1 = kf1_pose.GetOrigin();
  const Mat4d T_cw = kf2_pose.WorldToCamera();
  const Mat3d rot_2w = kf2_pose.RotationWorldToCamera();
  const Vec3d trans_2w = kf2_pose.TranslationWorldToCamera();
  // Compute relative position between the frames and project from kf1 to kf2
  // this has the benefit that we only have to compute the median depth once
  const Mat4d T_kf2_kf1 = T_cw * kf1_pose.CameraToWorld();
  const Mat3d R_kf2_kf1 = T_kf2_kf1.block<3, 3>(0, 0);
  const Vec3d t_kf2_kf1 = T_kf2_kf1.block<3, 1>(0, 3);
  size_t num_matches{0};
  std::vector<bool> is_already_matched_in_keyfrm_2(lms2.size(), false);
  // indices of KF 2 kpts in KF 1
  std::vector<int> matched_indices_2_in_keyfrm_1(lms1.size(), -1);
  
  const auto& cam1 = kf1.shot_camera_;
  const auto& cam2 = kf2.shot_camera_;
  
  Vec3d epiplane_in_keyfrm_2 = rot_2w * cam_center_1 + trans_2w;
  const Vec2d reproj =
      (cam2->GetProjectionMatrixScaled() * epiplane_in_keyfrm_2).hnormalized();
  epiplane_in_keyfrm_2.normalized();

  const Mat3d K = cam2->GetProjectionMatrixScaled();
  // for now, set to margin constant but vary
  // constexpr auto smargin{5}; //Experiment
  const auto inv_fx = 1.0 / K(0, 0);  // 1/fx;
  const auto inv_fy = 1.0 / K(1, 1);  // 1/fy;
  const auto cx = K(0, 2);            // cam->cx_p;
  const auto cy = K(1, 2);            // cam->cy_p;
  // Now, we draw the original points, then the matches in the other image
  // Then, we the reprojection and serach!
  const Mat3d KRK_i = cam1->GetProjectionMatrixScaled() * R_kf2_kf1 *
                      cam2->GetProjectionMatrixScaled().inverse();
  const Vec3d Kt = cam1->GetProjectionMatrixScaled() * t_kf2_kf1;
  const auto inv_min_depth  = 1.0f/min_depth;
  const auto inv_max_depth  = 1.0f/max_depth;
  const auto& undist_kpts1 = kf1.slam_data_.undist_keypts_;
  const auto& undist_kpts2 = kf2.slam_data_.undist_keypts_;
  // std::cout << "inv_min_depth: " << inv_min_depth << "/" << inv_max_depth << std::endl;
  for (size_t idx_1 = 0; idx_1 < lms1.size(); ++idx_1)
  {
    const auto* lm_1 = lms1[idx_1];
    if (lm_1 != nullptr) continue; // already matched to a landmark

    const auto& u_kpt_1 = undist_kpts1.at(idx_1);
    const float scale_1 = u_kpt_1.octave;
    AlignedVector<Vec2d> pts2D;
    if (scale_1 >= 0)
    {
      const Vec3d pt3D((u_kpt_1.point[0]-cx)*inv_fx, (u_kpt_1.point[1]-cy)*inv_fy, 1.0);
      const Vec3d ptTrans = KRK_i * Vec3d(u_kpt_1.point[0], u_kpt_1.point[1],1.0);
      const Vec2d start_pt = (ptTrans + Kt*inv_min_depth).hnormalized();
      const Vec2d end_pt = (ptTrans + Kt*inv_max_depth).hnormalized();
      const Vec2d epi_step = (end_pt-start_pt).normalized();

      //now, go through the epipolar line
      constexpr auto max_steps{50};
      Vec2d ptStep = start_pt; //+1.5*margin*step*epi_step;

      for (size_t step = 0; step < max_steps; ++step)
      {
          ptStep += epi_step*margin*1.5;
          if (grid_params_.in_grid(ptStep[0], ptStep[1]))
              pts2D.push_back(ptStep);
          else
              break; //Stop, once it's out of bounds!
          //TODO: Rethink the contFlag
          // if (start_pt[0] < end_pt[1])
          bool contFlag = (start_pt[0] < end_pt[0] ? ptStep[0] < end_pt[0] : ptStep[0] > end_pt[0]);
          contFlag = contFlag && (start_pt[1] < end_pt[1] ? ptStep[1] < end_pt[1] : ptStep[1] > end_pt[1]);
          // if (start_pt[0] > end_pt[1] && start_pt[1] > end_pt[1])
          //     break;
      }
    }

    std::unordered_set<size_t> unique_indices;
    //assemble the indices
    for (const auto& pt2D : pts2D)
    {
        const auto indices = GetKeypointsInCell(undist_kpts2, kf2.slam_data_.keypt_indices_in_cells_, pt2D[0], pt2D[1], margin);
        std::copy(indices.cbegin(), indices.cend(), std::inserter(unique_indices, unique_indices.end()));
    }
    const Vec3d& bearing_1 = kf1.slam_data_.bearings_.at(idx_1);
    const auto& desc_1 = kf1.GetDescriptor(idx_1);
    //use the guided matching instead of exhaustive
    auto best_hamm_dist = MAX_HAMMING_DIST;
    auto second_best_hamm_dist = MAX_HAMMING_DIST;
    int best_idx_2 = -1;
    auto n_epi_check_disc{0}, n_cos_disc{0};
    for (const auto idx_2 : unique_indices) 
    {
        const auto* lm_2 = lms2[idx_2];
        if (lm_2 != nullptr) continue; // already matched to a lm and not compatible
        if (is_already_matched_in_keyfrm_2.at(idx_2)) continue; //already matched to another feature
        const auto& desc_2 = kf2.GetDescriptor(idx_2);
        const auto hamm_dist = compute_descriptor_distance_32(desc_1, desc_2);

        if (HAMMING_DIST_THR_LOW < hamm_dist || best_hamm_dist < hamm_dist) {
            continue;
        }
        const Vec3d& bearing_2 = kf2.slam_data_.bearings_.at(idx_2);
        // If both are not stereo keypoints, don't use feature points near epipole
        const auto cos_dist = epiplane_in_keyfrm_2.dot(bearing_2);
        // Threshold angle between epipole and bearing (= 3.0deg)
        constexpr double cos_dist_thr = 0.99862953475;
        // do not match if the included angle is smaller than the threshold
        if (cos_dist_thr < cos_dist) continue; 
        else n_cos_disc++;

        // Consistency check with E matrix
        const bool is_inlier = CheckEpipolarConstraint(bearing_1, bearing_2, E_12,
                                                       scale_factors_.at(u_kpt_1.octave));
        if (is_inlier) {
            best_idx_2 = idx_2;
            best_hamm_dist = hamm_dist;
        }
        else
        {
            n_epi_check_disc++;
        }
    }
    if (best_idx_2 < 0) {
        continue;
    }

    is_already_matched_in_keyfrm_2.at(best_idx_2) = true;
    matched_indices_2_in_keyfrm_1.at(idx_1) = best_idx_2;
    ++num_matches;
  }

  matches.reserve(num_matches);
  //We do not check the orientation
  for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_keyfrm_1.size(); ++idx_1) {
      if (matched_indices_2_in_keyfrm_1.at(idx_1) >= 0) 
      {
        matches.emplace_back(std::make_pair(idx_1, matched_indices_2_in_keyfrm_1.at(idx_1)));
        if (idx_1 >= kf1.NumberOfKeyPoints() || matched_indices_2_in_keyfrm_1.at(idx_1) >= kf2.NumberOfKeyPoints())
        {
          std::cout << "OUT OF BOUNDS!" << idx_1 << "/" << matched_indices_2_in_keyfrm_1.at(idx_1) << " # tot: "
                    << kf1.NumberOfKeyPoints() << ", " << kf2.NumberOfKeyPoints() << std::endl;
          exit(0);
        }
      }
  }
  
  return matches;
}
bool 
GuidedMatcher::CheckEpipolarConstraint(const Vec3d& bearing_1, const Vec3d& bearing_2,
                                       const Mat3d& E_12, const float bearing_1_scale_factor)
{
  
  // normal vector of t epipolar plane on keyframe1
  const Vec3d epiplane_in_1 = E_12 * bearing_2;

  // Find the angle between normal vector and bearing
  const auto cos_residual = epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm();
  const auto residual_rad = M_PI / 2.0 - std::abs(std::acos(cos_residual));


  // threshold of inlier (=0.2deg)
  // (e.g. FOV=90deg, 900deg horizontal camera, 0.2deg corresponds to 2pix in horizontal direction)
  // TODO: Threshold parameterization
  constexpr double residual_deg_thr = 0.2;
  constexpr double residual_rad_thr = residual_deg_thr * M_PI / 180.0;

  // The larger the feature point scale, the looser the threshold
  // Consider TODO: threshold weighting
  return residual_rad < residual_rad_thr * bearing_1_scale_factor;
}


template<typename T> size_t 
GuidedMatcher::ReplaceDuplicatedLandmarks(map::Shot& fuse_shot, const T& landmarks_to_check, const float margin, map::Map& slam_map) const
{
  if (landmarks_to_check.empty())
  {
    std::cout << "landmarks_to_check: " << landmarks_to_check.size() << std::endl;
    return 0;
  }
  unsigned int num_fused = 0;
  size_t n_cam_to_lm_dist{0}, n_cam_disc{0};
  const auto& fuse_pose = fuse_shot.GetPose();
  const Mat3d R_cw = fuse_pose.RotationWorldToCamera();
  const Vec3d t_cw = fuse_pose.TranslationWorldToCamera();
  const Vec3d origin = fuse_pose.GetOrigin();
  const auto& cam = fuse_shot.shot_camera_;
  for (const auto& lm : landmarks_to_check)
  {
    // if the landmark is valid and doesn't see the fuse_shot yet
    // This happens with new triangulated features that have not
    // been matched the other frame
    if (lm != nullptr && !lm->IsObservedInShot(&fuse_shot))
    {
      const auto& lm_data = lm->slam_data_;
      const Vec3d& global_pos = lm->GetGlobalPos();
      const Vec2d pt2D = fuse_shot.ProjectInImageCoordinates(global_pos);
      {
        if (grid_params_.in_grid(pt2D[0], pt2D[1]))
        {
          constexpr auto ray_cos_thr{0.5};
          const Vec3d cam_to_lm_vec = global_pos - origin;
          const auto cam_to_lm_dist = cam_to_lm_vec.norm();
          // check if inside orb sale
          if (lm_data.GetMinValidDistance()  <= cam_to_lm_dist && 
              lm_data.GetMaxValidDistance() >= cam_to_lm_dist)
          {
            const Vec3d obs_mean_normal = lm_data.mean_normal_;
            const auto ray_cos = cam_to_lm_vec.dot(obs_mean_normal) / cam_to_lm_dist;
            if (ray_cos > ray_cos_thr)
            {
              const auto pred_scale_lvl = PredScaleLevel(lm_data.GetMaxValidDistance(), cam_to_lm_dist);
              // TODO: check scale level!
              // TODO: This part is very similar to the FindBestMatchForLandmark!!
              // This can probably be combined to a new function
              const auto indices = GetKeypointsInCell(fuse_shot.slam_data_.undist_keypts_,
                                                      fuse_shot.slam_data_.keypt_indices_in_cells_, 
                                                      pt2D[0], pt2D[1],
                                                      scale_factors_.at(pred_scale_lvl)*margin ,
                                                      pred_scale_lvl - 1, pred_scale_lvl + 1);
              if (!indices.empty())
              {
                const auto& lm_desc = lm->slam_data_.descriptor_;
                auto best_dist = MAX_HAMMING_DIST;
                auto best_idx = NO_MATCH;
                for (const auto idx : indices)
                {
                  const auto& keypt = fuse_shot.slam_data_.undist_keypts_.at(idx);
                  const size_t scale_level = keypt.octave;
                  if (scale_level >= pred_scale_lvl-1 && pred_scale_lvl >= scale_level)
                  {
                    const auto reproj_error_sq = (pt2D - keypt.point).squaredNorm();

                    // 自由度n=2
                    constexpr float chi_sq_2D = 5.99146;
                    
                    if (chi_sq_2D >= reproj_error_sq * inv_level_sigma_sq_.at(scale_level)) {
                      const auto& desc = fuse_shot.GetDescriptor(idx);
                      const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);
                      if (hamm_dist < best_dist)
                      {
                        best_dist = hamm_dist;
                        best_idx = idx;
                      }
                    }
                  }
                }
                //check if the match is valid
                if (best_dist <= HAMMING_DIST_THR_LOW)
                {
                  auto* shot_lm = fuse_shot.GetLandmark(best_idx);
                  //we already have a landmark there, thus either add or replace
                  if (shot_lm != nullptr) 
                  {
                    //replace the one with fewer observations by the other
                    if (lm->NumberOfObservations() < shot_lm->NumberOfObservations())
                    {
                      slam_map.ReplaceLandmark(lm, shot_lm); // replace lm by shot_lm
                      SlamUtilities::SetDescriptorFromObservations(*shot_lm);
                    }
                    else
                    {
                      slam_map.ReplaceLandmark(shot_lm, lm); // replace shot_lm by lm
                      SlamUtilities::SetDescriptorFromObservations(*lm);
                    }
                  }
                  else //add the observation
                  {
                    slam_map.AddObservation(&fuse_shot, lm, best_idx);
                  }
                  ++num_fused;
                }
              }
            }
          }
        }
      }
    }
  }
  // std::cout << "num_fused: " << num_fused << " num_cam_disc: " << n_cam_disc << " n_cam_to_lm_dist: " << n_cam_to_lm_dist << std::endl;
  return num_fused;
}


bool 
GuidedMatcher::IsObservable(map::Landmark* lm, const map::Shot& shot, const float ray_cos_thr,
                            Vec2d& reproj, size_t& pred_scale_level) const 
{
  
  const Vec3d pos_w = lm->GetGlobalPos();
  reproj = shot.Project(pos_w);
  const auto& lm_data = lm->slam_data_;
  if (shot.shot_camera_->CheckWithinBoundaries(reproj))
  {
    if (grid_params_.in_grid(reproj[0], reproj[1]))
    {
      const Vec3d cam_to_lm_vec = pos_w - shot.GetPose().GetOrigin();
      const auto cam_to_lm_dist = cam_to_lm_vec.norm();
      // check if inside orb sale
      if (lm_data.GetMinValidDistance()  <= cam_to_lm_dist && 
          lm_data.GetMaxValidDistance() >= cam_to_lm_dist)
      {
        const Vec3d obs_mean_normal = lm_data.mean_normal_;
        const auto ray_cos = cam_to_lm_vec.dot(obs_mean_normal) / cam_to_lm_dist;
        if (ray_cos > ray_cos_thr)
        {
          pred_scale_level = PredScaleLevel(lm_data.GetMaxValidDistance(), cam_to_lm_dist);
          return true;
        }
      }
    }
  }
  return false;
}

size_t 
GuidedMatcher::PredScaleLevel(const float max_valid_dist, const float cam_to_lm_dist) const
{
  const auto ratio = max_valid_dist / cam_to_lm_dist;
  const auto pred_scale_level = static_cast<int>(std::ceil(std::log(ratio) / log_scale_factor_));
  if (pred_scale_level < 0) return 0;
  if (num_scale_levels_ <= static_cast<unsigned int>(pred_scale_level)) return num_scale_levels_ - 1;
  return static_cast<unsigned int>(pred_scale_level);
}
template size_t GuidedMatcher::ReplaceDuplicatedLandmarks(map::Shot&, const std::vector<map::Landmark*>&, const float, map::Map& slam_map) const;
template size_t GuidedMatcher::ReplaceDuplicatedLandmarks(map::Shot&, const std::unordered_set<map::Landmark*>&, const float, map::Map& slam_map) const;
template size_t GuidedMatcher::ReplaceDuplicatedLandmarks(map::Shot&, const std::set<map::Landmark*, map::KeyCompare>&, const float, map::Map& slam_map) const;

}; // namespace slam
