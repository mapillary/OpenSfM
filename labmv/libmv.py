import numpy
import ctypes

libmvc = ctypes.cdll.LoadLibrary('../lib/build/libmvc.dylib')

libmvc.bar.restype = ctypes.c_int
libmvc.bar.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
def bar(x):
    y = x.copy()
    return libmvc.bar(y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), len(y))



x1, x2, FLAGS_threshold, &F, &inliers

x = numpy.arange(10, dtype=float)
n = bar(x[::2])