#include <slam/slam_utilities.h>
#include <slam/slam_map.h>
#include <map/shot.h>
#include <map/landmark.h>
namespace slam
{

void SlamMap::UpdateLandmarksAfterKfInsert(map::Shot *shot)
{
  const auto &landmarks = shot->GetLandmarks();
  const auto n_landmarks = landmarks.size();
  for (unsigned int feat_id = 0; feat_id < n_landmarks; ++feat_id)
  {
    // auto lm = new_kf->landmarks_.at(feat_id);
    auto *lm = landmarks.at(feat_id);
    if (lm == nullptr)
    {
      // if `lm` is correctly observed, make it be checked by the local map cleaner
      // local_map_cleaner_->add_fresh_landmark(lm);
      if (lm->IsObservedInShot(shot))
      {

        fresh_landmarks_.push_back(lm);
        std::cout << "Actually inserted something!" << shot->id_ << std::endl;
        continue;
      }
      // if `lm` does not have the observation information from `cur_keyfrm_`,
      // add the association between the keyframe and the landmark
      else
      {
        lm->AddObservation(shot, feat_id);
        SlamUtilities::SetDescriptorFromObservations(*lm);
        // SlamUtilities::SetNormalAndDepthFromObservations(*lm, scale);
      }
    }
  }
  // std::cout << "Before update connections!" << std::endl;
  shot->slam_data_.graph_node_->update_connections();
}

/*
size_t
SlamMap::remove_redundant_landmarks(const size_t cur_keyfrm_id) 
{
    constexpr float observed_ratio_thr = 0.3;
    constexpr unsigned int num_reliable_keyfrms = 2;
    // const unsigned int num_obs_thr = is_monocular_ ? 2 : 3;
    constexpr size_t num_obs_thr{2};

    // states of observed landmarks
    enum class lm_state_t { Valid, Invalid, NotClear };

    unsigned int num_removed = 0;
    auto iter = fresh_landmarks_.begin();
    while (iter != fresh_landmarks_.end()) {
        auto lm = *iter;

        // decide the state of lms the buffer
        auto lm_state = lm_state_t::NotClear;
        if (lm->will_be_erased()) {
            // in case `lm` will be erased
            // remove `lm` from the buffer
            lm_state = lm_state_t::Valid;
        }
        else if (lm->get_observed_ratio() < observed_ratio_thr) {
            // if `lm` is not reliable
            // remove `lm` from the buffer and the database
            lm_state = lm_state_t::Invalid;
        }
        else if (num_reliable_keyfrms + lm->ref_kf_id_ <= cur_keyfrm_id
                 && lm->num_observations() <= num_obs_thr) {
            // if the number of the observers of `lm` is small after some keyframes were inserted
            // remove `lm` from the buffer and the database
            lm_state = lm_state_t::Invalid;
        }
        else if (num_reliable_keyfrms + 1U + lm->ref_kf_id_ <= cur_keyfrm_id) {
            // if the number of the observers of `lm` is sufficient after some keyframes were inserted
            // remove `lm` from the buffer
            lm_state = lm_state_t::Valid;
        }

        // select to remove `lm` according to the state
        if (lm_state == lm_state_t::Valid) {
            iter = fresh_landmarks_.erase(iter);
        }
        else if (lm_state == lm_state_t::Invalid) {
            ++num_removed;
            lm->prepare_for_erasing();
            iter = fresh_landmarks_.erase(iter);
            map_db_->erase_landmark(lm);
        }
        else {
            // hold decision because the state is NotClear
            iter++;
        }
    }
    std::cout << "remove_redundant_landmarks: " << num_removed << std::endl;
    return num_removed;
}


size_t
SlamMap::remove_redundant_keyframes(Shot* cur_keyfrm, const size_t origin_kf_id) 
{
    // window size not to remove
    constexpr unsigned int window_size_not_to_remove = 2;
    // if the redundancy ratio of observations is larger than this threshold,
    // the corresponding keyframe will be erased
    constexpr float redundant_obs_ratio_thr = 0.9;

    size_t num_removed = 0;
    // check redundancy for each of the covisibilities
    const auto cur_covisibilities = cur_keyfrm->graph_node_->get_covisibilities();
    for (const auto covisibility : cur_covisibilities) {
        // cannot remove the origin
        if (covisibility->kf_id_ == origin_kf_id) {
            continue;
        }
        // cannot remove the recent keyframe(s)
        if (covisibility->kf_id_ <= cur_keyfrm->kf_id_
            && cur_keyfrm->kf_id_ <= covisibility->kf_id_ + window_size_not_to_remove) {
            continue;
        }

        // count the number of redundant observations (num_redundant_obs) and valid observations (num_valid_obs)
        // for the covisibility
        size_t num_redundant_obs{0}, num_valid_obs{0};
        // unsigned int num_valid_obs = 0;
        count_redundant_observations(covisibility, num_valid_obs, num_redundant_obs);
        std::cout << covisibility->im_name_ << " num_redundant_obs" << num_redundant_obs 
                  << "num_valid_obs" << num_valid_obs << " factor: " << static_cast<float>(num_redundant_obs) / num_valid_obs << std::endl;
        // if the redundant observation ratio of `covisibility` is larger than the threshold, it will be removed
        if (redundant_obs_ratio_thr <= static_cast<float>(num_redundant_obs) / num_valid_obs) {
            ++num_removed;
            covisibility->prepare_for_erasing(*map_db_);
        }
    }

    return num_removed;
}*/



/*
void SlamMap::InsertNewKeyFrame(map::Shot* shot)
{
    //Make a shot to a keyframe
    auto& landmarks = shot->GetLandmarks();
    const auto n_landmarks{landmarks.size()};
    for (size_t feat_id = 0; feat_id < n_landmarks; ++feat_id)
    {
        auto* lm = landmarks[feat_id];
        if (lm != nullptr)
        {
            //If it is observer it comes from triangulation
            if (lm->IsObservedInShot(shot))
            {

            }
            else //else from tracking
            {
                lm->AddObservation(shot, feat_id);

            }
        }
    }
}

    // landmarks_
    for (unsigned int idx = 0; idx < new_kf->landmarks_.size(); ++idx) {
        auto lm = new_kf->landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // if `lm` does not have the observation information from `cur_keyfrm_`,
        // add the association between the keyframe and the landmark
        if (lm->is_observed_in_keyframe(new_kf)) {
            // if `lm` is correctly observed, make it be checked by the local map cleaner
            // local_map_cleaner_->add_fresh_landmark(lm);
            fresh_landmarks_.push_back(lm);
            std::cout << "Actually inserted something!"
            continue;
        }

        // update connection
        lm->add_observation(new_kf, idx);
        // update geometry
        lm->update_normal_and_depth();
        lm->compute_descriptor();
    }
    // std::cout << "Before update connections!" << std::endl;
    new_kf->graph_node_->update_connections();
    // std::cout << "AFter update connections!" << std::endl;
    const auto cov = new_kf->graph_node_->get_top_n_covisibilities(10);


*/

} // namespace slam