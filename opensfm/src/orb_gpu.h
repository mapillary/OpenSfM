#include "types.h"

#include <opencv2/core.hpp>
#include <opencv2/cudafeatures2d.hpp>

namespace csfm {

int CUDA_getCudaEnabledDeviceCount() {
  return cv::cuda::getCudaEnabledDeviceCount();
}

void CUDA_printShortCudaDeviceInfo(const int device_id) {
  cv::cuda::printShortCudaDeviceInfo(device_id);
}

bool CUDA_setDevice(const int device_id) {
  cv::cuda::setDevice(device_id);
  cv::cuda::DeviceInfo dev_info(device_id);
  if (dev_info.isCompatible()) {
    CUDA_printShortCudaDeviceInfo(device_id);
  } else {
    std::cout << "CUDA module isn't built for GPU #" << device_id << " ("
              << dev_info.name() << ", CC " << dev_info.majorVersion()
              << dev_info.minorVersion() << "\n";
    return false;
  }
  return true;
}

class OrbGpu
{
public:
  OrbGpu(const int nfeatures, 
         const float scaleFactor, 
         const int nlevels,
         const int edgeThreshold,
         const int firstLevel,
         const int WTA_K,
         const int scoreType,
         const int patchSize,
         const int fastThreshold,
         const bool blurForDescriptor) {

    std::cout << "Loading ORB-GPU ..." << std::endl;

    // Instantiate ORB GPU object
    orb_ = cv::cuda::ORB::create(nfeatures,
                                 scaleFactor,
                                 nlevels,
                                 edgeThreshold,
                                 firstLevel,
                                 WTA_K,
                                 scoreType, 
                                 patchSize, 
                                 fastThreshold,
                                 blurForDescriptor);

    std::cout << "Loading ORB-GPU ... [OK]" << std::endl;

  }

  //OrbGpu(const OrbGpu &);
  
  bp::object detectAndCompute(PyObject *image);

private:
  cv::Ptr<cv::cuda::ORB> orb_;
};


bp::object OrbGpu::detectAndCompute(PyObject *image) {
  PyArrayContiguousView<unsigned char> view((PyArrayObject *)image);
  const cv::Mat img(view.shape(0), view.shape(1), CV_8U, (void *)view.data());

  const cv::cuda::GpuMat d_img(img);

  cv::cuda::GpuMat desc_gpu;
  std::vector<cv::KeyPoint> kpts_cpu;

  // Detect ORB keypoints and extract descriptors on image
  orb_->detectAndCompute(d_img, cv::cuda::GpuMat(), kpts_cpu, desc_gpu);

  // GPU -> CPU
  cv::Mat desc_cpu(desc_gpu);

  // Convert to numpy.
  cv::Mat keys(kpts_cpu.size(), 4, CV_32F);
  for (size_t i = 0; i < kpts_cpu.size(); ++i) {
    keys.at<float>(i, 0) = kpts_cpu[i].pt.x;
    keys.at<float>(i, 1) = kpts_cpu[i].pt.y;
    keys.at<float>(i, 2) = kpts_cpu[i].size;
    keys.at<float>(i, 3) = kpts_cpu[i].angle;
  }

  bp::list retn;
  npy_intp keys_shape[2] = {keys.rows, keys.cols};
  retn.append(bpn_array_from_data(2, keys_shape, keys.ptr<float>(0)));
  npy_intp desc_shape[2] = {desc_cpu.rows, desc_cpu.cols};
  retn.append(bpn_array_from_data(2, desc_shape, desc_cpu.ptr<uchar>(0)));
  return retn;
}


}
