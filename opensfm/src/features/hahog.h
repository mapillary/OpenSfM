#ifndef __HAHOG_H__
#define __HAHOG_H__

#include <foundation/types.h>

namespace features {

py::object hahog(foundation::pyarray_f image,
                 float peak_threshold,
                 float edge_threshold,
                 int target_num_features,
                 bool use_adaptive_suppression);

}

#endif // __HAHOG_H__
