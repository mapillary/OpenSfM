#include "types.h"
#include "AKAZE.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace csfm {


bp::object akaze(PyObject *image,
                 AKAZEOptions options) {
  PyArrayContiguousView<unsigned char> view((PyArrayObject *)image);
  const cv::Mat img(view.shape(0), view.shape(1), CV_8U, (void *)view.data());

  cv::Mat img_32;
  img.convertTo(img_32, CV_32F, 1.0 / 255.0, 0);

  // Don't forget to specify image dimensions in AKAZE's options
  options.img_width = img_32.cols;
  options.img_height = img_32.rows;

  // Extract features
  libAKAZE::AKAZE evolution(options);
  std::vector<cv::KeyPoint> kpts;

  evolution.Create_Nonlinear_Scale_Space(img_32);
  evolution.Feature_Detection(kpts);

  // Compute descriptors.
  cv::Mat desc;
  evolution.Compute_Descriptors(kpts, desc);

  evolution.Show_Computation_Times();

  // Convert to numpy.
  cv::Mat keys(kpts.size(), 4, CV_32F);
  for (int i = 0; i < (int) kpts.size(); ++i) {
    keys.at<float>(i, 0) = kpts[i].pt.x;
    keys.at<float>(i, 1) = kpts[i].pt.y;
    keys.at<float>(i, 2) = kpts[i].size;
    keys.at<float>(i, 3) = kpts[i].angle;
  }

  bp::list retn;
  retn.append(bpn_array_from_data(keys.ptr<float>(0), keys.rows, keys.cols));

  if (options.descriptor == MLDB_UPRIGHT || options.descriptor == MLDB) {
    retn.append(bpn_array_from_data(desc.ptr<unsigned char>(0), desc.rows, desc.cols));
  } else {
    retn.append(bpn_array_from_data(desc.ptr<float>(0), desc.rows, desc.cols));
  }
  return retn;
}


}
