#ifndef __TYPES_H__
#define __TYPES_H__

#include <boost/python/numpy.hpp>
#include <vector>
#include <iostream>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h> 

namespace csfm {

namespace bp = boost::python;
namespace bpn = boost::python::numpy;

template <typename T> inline int numpy_typenum() {}
template <> inline int numpy_typenum<bool>() { return NPY_BOOL; }
template <> inline int numpy_typenum<char>() { return NPY_INT8; }
template <> inline int numpy_typenum<short>() { return NPY_INT16; }
template <> inline int numpy_typenum<int>() { return NPY_INT32; }
template <> inline int numpy_typenum<long long>() { return NPY_INT64; }
template <> inline int numpy_typenum<unsigned char>() { return NPY_UINT8; }
template <> inline int numpy_typenum<unsigned short>() { return NPY_UINT16; }
template <> inline int numpy_typenum<unsigned int>() { return NPY_UINT32; }
template <> inline int numpy_typenum<unsigned long long>() { return NPY_UINT64; }
template <> inline int numpy_typenum<float>() { return NPY_FLOAT32; }
template <> inline int numpy_typenum<double>() { return NPY_FLOAT64; }

template <typename T> inline const char *type_string() {}
template <> inline const char *type_string<bool>() { return "bool"; }
template <> inline const char *type_string<char>() { return "int8"; }
template <> inline const char *type_string<short>() { return "int16"; }
template <> inline const char *type_string<int>() { return "int32"; }
template <> inline const char *type_string<long long>() { return "int64"; }
template <> inline const char *type_string<unsigned char>() { return "uint8"; }
template <> inline const char *type_string<unsigned short>() { return "uint16"; }
template <> inline const char *type_string<unsigned int>() { return "uint32"; }
template <> inline const char *type_string<unsigned long long>() { return "uint64"; }
template <> inline const char *type_string<float>() { return "float32"; }
template <> inline const char *type_string<double>() { return "float64"; }

template <typename T>
bp::object bpn_array_from_data(int nd, npy_intp *shape, const T *data) {
  if(nd == 1) {
    return bpn::from_data((void*)data, bpn::dtype::get_builtin<T>(), bp::make_tuple(shape[0]), bp::make_tuple(sizeof(T)), bp::object()).copy();
  } else if(nd == 2) {
    return bpn::from_data((void*)data, bpn::dtype::get_builtin<T>(), bp::make_tuple(shape[0], shape[1]), bp::make_tuple(sizeof(T)*shape[1], sizeof(T)), bp::object()).copy();
  } else {
    assert(nd > 0 && nd < 3);
  }
}

template <typename T>
bp::object bpn_array_from_vector(const std::vector<T> &v) {
  npy_intp shape[] = { v.size() };
  const T *data = v.size() ? &v[0] : NULL;
  return bpn_array_from_data(1, shape, data);
}

template<typename T>
class PyArrayContiguousView {
 public:
  PyArrayContiguousView(bpn::ndarray array) {
    init((PyArrayObject *)array.ptr());
  }

  PyArrayContiguousView(bp::object object) {
    init((PyArrayObject *)object.ptr());
  }

  PyArrayContiguousView(PyArrayObject *object) {
    init(object);
  }

  ~PyArrayContiguousView() {
    if (contiguous_) Py_DECREF(contiguous_);
  }

  const T *data() const {
    return (T *)PyArray_DATA(contiguous_);
  }

  int ndim() const {
    return PyArray_NDIM(contiguous_);
  }

  int shape(int dim) const {
    return PyArray_DIMS(contiguous_)[dim];
  }

  bool valid() {
    return contiguous_ != NULL;
  }

 private:
  void init(PyArrayObject *object) {
    int type = PyArray_DESCR(object)->type_num;
    if (type != numpy_typenum<T>()) {
      std::cerr << "Error in PyArrayContiguousView: expected array of type " << type_string<T>() << std::endl;
      contiguous_ = NULL;
    } else {
      contiguous_ = (PyArrayObject *)PyArray_ContiguousFromAny((PyObject *)object, numpy_typenum<T>(), 0, 0);
    }
  };

  PyArrayObject *contiguous_;
};

}

#endif // __TYPES_H__
