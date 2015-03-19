#ifndef __DSIFT_H__
#define __DSIFT_H__

#include "types.h"

namespace csfm {

bp::object dsift(PyObject *image,
                 int step,
                 int bin_size);

}

#endif // __DSIFT_H__
