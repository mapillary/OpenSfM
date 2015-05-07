#ifndef LIBMV_MULTIVIEW_POSE_KNOWN_ROTATION_ROBUST_H_
#define LIBMV_MULTIVIEW_POSE_KNOWN_ROTATION_ROBUST_H_

#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"

namespace libmv {


double PoseKnownRotationRobust(const Mat &v,
                                const Mat &X,
                                double threshold,
                                Vec3 *c,
                                vector<int> *inliers);


} // namespace libmv

#endif  // LIBMV_MULTIVIEW_POSE_KNOWN_ROTATION_ROBUST_H_