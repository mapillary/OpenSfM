#include "types.h"
#include "dsift.h"

#include <vector>
#include <iostream>

extern "C" {
  #include "vl/covdet.h"
  #include "vl/dsift.h"
  #include "vl/sift.h"
  #include <time.h>
}


namespace csfm {

bp::object dsift (PyObject *image,
                 int step,
                 int bin_size)
{
  PyArrayContiguousView<float> im((PyArrayObject *)image);

  if (im.valid()) {
    clock_t t_start = clock();

    VlDsiftFilter *dsift = vl_dsift_new_basic (im.shape(1), im.shape(0), step, bin_size);
    vl_dsift_process(dsift, im.data());

    int numFeatures = vl_dsift_get_keypoint_num(dsift);
    // extract keypoints
    VlDsiftKeypoint const * dsift_keypoints = vl_dsift_get_keypoints (dsift);
    #define DIM 4
    std::vector<float> points(DIM * numFeatures);
    for (int i = 0; i < numFeatures; ++i) {
      points [DIM * i + 0] = dsift_keypoints[i].x;
      points [DIM * i + 1] = dsift_keypoints[i].y;
      points [DIM * i + 2] = dsift_keypoints[i].s;
      points [DIM * i + 3] = dsift_keypoints[i].norm;
    }
    // get descriptors
    int sizeDescriptor = vl_dsift_get_descriptor_size(dsift);
    float const *dsift_descriptors = vl_dsift_get_descriptors(dsift);

    clock_t t_end = clock();
    std::cout << "dsift number features " << numFeatures << "\n";
    std::cout << "dsift descriptor size " << sizeDescriptor << "\n";
    std::cout << "dsift step " << step << " bin_size " << bin_size << "\n";
    std::cout << "dsift time " << float(t_end - t_start)/CLOCKS_PER_SEC << "\n";

    bp::list retn;
    npy_intp points_shape[2] = {npy_intp(numFeatures), DIM};
    retn.append(bpn_array_from_data(2, points_shape, &points[0]));

    npy_intp desc_shape[2] = {npy_intp(numFeatures), sizeDescriptor};
    retn.append(bpn_array_from_data(2, desc_shape, dsift_descriptors));

    vl_dsift_delete(dsift);
    return retn;
  }
  return bp::object();
}

}
