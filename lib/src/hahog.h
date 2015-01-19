#ifndef __HAHOG_H__
#define __HAHOG_H__

#include "types.h"

namespace csfm {

bp::object hahog(PyObject *image,
                 float peak_threshold,
                 float edge_threshold);
}

#endif // __HAHOG_H__
