#pragma once

#include <foundation/types.h>
#include <third_party/akaze/lib/AKAZE.h>

namespace features {

py::object akaze(foundation::pyarray_uint8 image, AKAZEOptions options);

}
