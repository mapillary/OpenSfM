#ifndef __HAHOG_H__
#define __HAHOG_H__

#include "types.h"

namespace csfm {

bp::object hahog(PyObject *image,
                 float peak_threshold,
<<<<<<< HEAD
                 float edge_threshold);
=======
                 float edge_threshold,
                 int target_num_features,
                 bool use_adaptive_suppression);
>>>>>>> upstream/master

}

#endif // __HAHOG_H__
