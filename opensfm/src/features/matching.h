#pragma once

#include <foundation/python_types.h>

namespace features {

py::object match_using_words(foundation::pyarray_f features1,
                             foundation::pyarray_int words1,
                             foundation::pyarray_f features2,
                             foundation::pyarray_int words2, float lowes_ratio,
                             int max_checks);

}
