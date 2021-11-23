#pragma once

#include <foundation/python_types.h>
#include <third_party/akaze/lib/AKAZE.h>

namespace features {

py::tuple akaze(foundation::pyarray_uint8 image, AKAZEOptions options);

}
