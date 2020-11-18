#pragma once

#include <foundation/python_types.h>

namespace features {

py::object hahog(foundation::pyarray_f image, float peak_threshold,
                 float edge_threshold, int target_num_features,
                 bool use_adaptive_suppression);

}
