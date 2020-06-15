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
} // namespace slam