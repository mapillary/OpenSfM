#pragma once
#include <slam/third_party/orb_extractor/orb_extractor.h>
#include <foundation/python_types.h>
#include <map/shot.h>
#include <opencv2/core.hpp>
namespace py = pybind11;
namespace slam
{
class OrbExtractorWrapper
{
public:
  OrbExtractorWrapper() = delete;
  OrbExtractorWrapper(const unsigned int max_num_keypts, const float scale_factor, const unsigned int num_levels,
                      const unsigned int ini_fast_thr, const unsigned int min_fast_thr):
                      extractor_(max_num_keypts, scale_factor, num_levels, ini_fast_thr, min_fast_thr)
  {
    // extractor_ = std::make_unique<openvslam::feature::orb_extractor>(max_num_keypts, scale_factor, num_levels, ini_fast_thr, min_fast_thr);
  }

  void extract_to_shot(map::Shot& shot, foundation::pyarray_uint8 image, foundation::pyarray_uint8 mask)
  {
    const cv::Mat img(image.shape(0), image.shape(1), CV_8U, (void *)image.data());
    const cv::Mat mask_img = (mask.shape(0) == 0 ? cv::Mat{} : cv::Mat(mask.shape(0), mask.shape(1), CV_8U, (void *)mask.data()));
    AlignedVector<Observation> kpts;
    DescriptorMatrix desc;
    // cv::Mat desc;
    
    extractor_.extract(img, mask_img, kpts, desc);//, dmat);
    shot.InitAndTakeDatastructures(kpts, desc);
    // std::swap(dmat, shot.descriptors_eig_);
  }

  // py::list extract(foundation::pyarray_uint8 image, foundation::pyarray_uint8 mask)
  // {
  //   const cv::Mat img(image.shape(0), image.shape(1), CV_8U, (void *)image.data());
  //   const cv::Mat mask_img = (mask.shape(0) == 0 ? cv::Mat{} : cv::Mat(mask.shape(0), mask.shape(1), CV_8U, (void *)mask.data()));
  //   std::vector<cv::KeyPoint> kpts;
  //   cv::Mat desc;
  //   extractor_.extract(img, mask_img, kpts, desc);
  //   cv::Mat keys(kpts.size(), 5, CV_32F);
  //   for (int i = 0; i < (int) kpts.size(); ++i) {
  //       keys.at<float>(i, 0) = kpts[i].pt.x;
  //       keys.at<float>(i, 1) = kpts[i].pt.y;
  //       keys.at<float>(i, 2) = kpts[i].size;
  //       keys.at<float>(i, 3) = kpts[i].angle;
  //       keys.at<float>(i, 4) = kpts[i].octave;
  //   }

  //   py::list retn;
  //   retn.append(foundation::py_array_from_data(keys.ptr<float>(0), keys.rows, keys.cols));
  //   retn.append(foundation::py_array_from_data(desc.ptr<unsigned char>(0), desc.rows, desc.cols));
  //   return retn;
  // }

  py::list extract(foundation::pyarray_uint8 image, foundation::pyarray_uint8 mask)
  {
    const cv::Mat img(image.shape(0), image.shape(1), CV_8U, (void *)image.data());
    const cv::Mat mask_img = (mask.shape(0) == 0 ? cv::Mat{} : cv::Mat(mask.shape(0), mask.shape(1), CV_8U, (void *)mask.data()));
    // std::vector<cv::KeyPoint> kpts;
    AlignedVector<Observation> kpts;
    DescriptorMatrix desc;
    // cv::Mat desc;
    extractor_.extract(img, mask_img, kpts, desc); //, dmat);
    // cv::Mat keys(kpts.size(), 5, CV_32F);
    // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> keys(kpts.size(), 5);
    // for (int i = 0; i < (int) kpts.size(); ++i) {
    //     keys
    //     // keys.at<float>(i, 0) = kpts[i].point[0];
    //     // keys.at<float>(i, 1) = kpts[i].point[1];
    //     // keys.at<float>(i, 2) = kpts[i].size;
    //     // keys.at<float>(i, 3) = kpts[i].angle;
    //     // keys.at<float>(i, 4) = kpts[i].scale;
    // }

    py::list retn;
    // retn.append(foundation::py_array_from_data(keys.ptr<float>(0), keys.rows, keys.cols));
    retn.append(kpts);
    // retn.append(foundation::py_array_from_data(desc.ptr<unsigned char>(0), desc.rows, desc.cols));
    retn.append(desc);
    return retn;
  }


  const std::vector<float>
  GetScaleLevels() const { return extractor_.get_scale_factors(); }
private:
  openvslam::feature::orb_extractor extractor_;
};  
} //namespace slam