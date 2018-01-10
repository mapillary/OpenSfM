#include "types.h"
#include "dsift.h"

#include <vector>
#include <iostream>

extern "C" {
  #include "vl/dsift.h"
  #include "vl/imopv.h"
  #include <time.h>
}


namespace csfm {

bp::object dsift(PyObject *image,
		 int step,
		 int bin_size,
		 bool use_flat_window) {
  PyArrayContiguousView<float> im((PyArrayObject *)image);

  if (im.valid()) {
    VlDsiftFilter *dsift = vl_dsift_new_basic(im.shape(1), im.shape(0), step, bin_size);
    vl_dsift_set_flat_window(dsift, use_flat_window);
    vl_dsift_process(dsift, im.data());
    int const numkp = vl_dsift_get_keypoint_num(dsift);
    VlDsiftKeypoint const * keypoints = vl_dsift_get_keypoints(dsift);
    std::vector<float> points(4 * numkp);
    for (int i = 0; i < numkp; ++i) {
      VlDsiftKeypoint const & kp = keypoints[i];
      points[4 * i + 0] = (float) kp.x;
      points[4 * i + 1] = (float) kp.y;
      points[4 * i + 2] = (float) kp.s;
      points[4 * i + 3] = (float) kp.norm;
    }
    // Get the descriptors: size is descsize * numkp:
    int const descsize = vl_dsift_get_descriptor_size(dsift);
    std::vector<float> desc(descsize * numkp);
    float const *descriptors = vl_dsift_get_descriptors(dsift);
    bp::list retn;
    npy_intp points_shape[2] = {npy_intp(numkp), 4};
    retn.append(bpn_array_from_data(2, points_shape, &points[0]));
    npy_intp desc_shape[2] = {npy_intp(numkp), npy_intp(descsize)};
    retn.append(bpn_array_from_data(2, desc_shape, descriptors));

    vl_dsift_delete(dsift);
    return retn;
  }
  return bp::object();
}

}
