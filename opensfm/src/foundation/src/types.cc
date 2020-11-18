#include <foundation/python_types.h>

namespace foundation {

cv::Mat pyarray_cv_mat_view(pyarray_f &array) {
  return pyarray_cv_mat_view_typed(array, CV_32F);
}

cv::Mat pyarray_cv_mat_view(pyarray_d &array) {
  return pyarray_cv_mat_view_typed(array, CV_64F);
}

cv::Mat pyarray_cv_mat_view(pyarray_int &array) {
  return pyarray_cv_mat_view_typed(array, CV_32S);
}

cv::Mat pyarray_cv_mat_view(pyarray_uint8 &array) {
  return pyarray_cv_mat_view_typed(array, CV_8U);
}

}  // namespace foundation
