#include "types.h"

#include <opencv2/core.hpp>
#include <opencv2/cudafeatures2d.hpp>

namespace csfm {

/*struct DMatch
{
  float distance;
  float imgIdx;
  float queryIdx;
  float trainIdx;
};*/

class BFMatcherGpu
{
public:
  BFMatcherGpu(const int normType) {

    std::cout << "Loading BFMatcherGpu ..." << std::endl;

    // Instantiate BFMatcherGpu object
    bfm_ = cv::cuda::DescriptorMatcher::createBFMatcher(normType);

    std::cout << "Loading BFMatcherGpu ... [OK]" << std::endl;

  }
  
  bp::list knnMatch(PyObject *queryDescriptors,
                      PyObject *trainDescriptors,
                      const int k);

private:
  cv::Ptr<cv::cuda::DescriptorMatcher> bfm_;
};


bp::list BFMatcherGpu::knnMatch(PyObject *queryDesc_,
                                  PyObject *trainDesc_,
                                  const int k) {
  PyArrayContiguousView<float> query_desc_((PyArrayObject *)queryDesc_),
                               train_desc_((PyArrayObject *)trainDesc_);

  const cv::Mat query_desc(query_desc_.shape(0), query_desc_.shape(1), 
                           CV_32F, (void *)query_desc_.data()),
                train_desc(train_desc_.shape(0), train_desc_.shape(1), 
                           CV_32F, (void *)train_desc_.data());

  cv::Mat query_desc_8u, train_desc_8u;
  query_desc.convertTo(query_desc_8u, CV_8U);
  train_desc.convertTo(train_desc_8u, CV_8U);

  std::vector<std::vector<cv::DMatch> > matches_cpu;
  const cv::cuda::GpuMat d_query_desc(query_desc_8u), 
                         d_train_desc(train_desc_8u);
  
  // Perform Brute-Force matching
  bfm_->knnMatch(d_query_desc, d_train_desc, matches_cpu, k);

  //std::cout << matches_cpu.size() << std::endl;

  // Convert to numpy.
  bp::list retn;
  for (size_t i = 0; i < matches_cpu.size(); ++i)
    retn.append(std_vector_to_py_list(matches_cpu[i]));

  return retn;
}


}
