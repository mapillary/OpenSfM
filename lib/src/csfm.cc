#include <Python.h> // This must be included before anything else. See http://bugs.python.org/issue10910
#include <boost/python.hpp>
#include <iostream>
#include <vector>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h> 

#include "types.h"
#include "hahog.cc"


BOOST_PYTHON_MODULE(csfm) {
  using namespace boost::python;
  boost::python::numeric::array::set_module_and_type("numpy", "ndarray");
  import_array();

  // Add regular functions to the module.
  def("hahog", csfm::hahog,
      (arg("peak_threshold") = 0.003,
       arg("edge_threshold") = 10
      )
  );
}
