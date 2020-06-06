#pragma once
#include <map/map.h>
#include <Eigen/Eigen>

namespace slam
{
class SlamMap : public map::Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // void InsertNewKeyFrame(Shot* shot);
    void UpdateLandmarksAfterKfInsert(map::Shot *shot);
    void RemoveRedundantLandmarks(const map::ShotId cur_keyfrm_id)
    {
        // TODO: Implement
    }
    void RemoveRedundantKeyFrames(map::Shot* cur_keyfrm, const map::ShotId origin_kf_id)
    {
        // TODO: Implement
    }
private:
    // std::unordered_map<ShotId, Shot*> keyframes_;
    std::vector<map::Landmark*> fresh_landmarks_;
};
}