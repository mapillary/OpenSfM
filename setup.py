#!/usr/bin/env python

from distutils.core import setup, Extension
import numpy as np
import os
import glob

default_include_dirs = [
    '/usr/local/include',
    '/usr/include',
]

cplus_include_path = os.getenv('CPLUS_INCLUDE_PATH', '')
if cplus_include_path:
    default_include_dirs.extend(cplus_include_path.split(':'))


def find_path(name, hints, path_suffixes=[]):
    suffixes = [''] + path_suffixes
    for c in hints:
        for s in suffixes:
            path = os.path.join(c, s)
            if os.path.isfile(os.path.join(path, name)):
                return path
    return None

def find_include(name, hints, path_suffixes=[]):
    return find_path(name, hints + default_include_dirs, path_suffixes)


libraries = []
library_dirs = ['/usr/local/lib']

# Eigen
eigen_include_dir = find_include('Eigen/Core', [], ['eigen3'])

# Ceres
ceres_libraries = ['ceres', 'glog', 'gflags']
libraries.extend(ceres_libraries)

# Boost Python
boost_python_libraries = ['boost_python']
libraries.extend(boost_python_libraries)

# Akaze
akaze_include_dir = 'opensfm/src/third_party/akaze/lib'
akaze_sources = [
    'opensfm/src/third_party/akaze/lib/AKAZE.cpp',
    'opensfm/src/third_party/akaze/lib/fed.cpp',
    'opensfm/src/third_party/akaze/lib/nldiffusion_functions.cpp',
    'opensfm/src/third_party/akaze/lib/utils.cpp',
]
akaze_depends = [
    'opensfm/src/third_party/akaze/lib/AKAZEConfig.h',
    'opensfm/src/third_party/akaze/lib/AKAZE.h',
    'opensfm/src/third_party/akaze/lib/fed.h',
    'opensfm/src/third_party/akaze/lib/nldiffusion_functions.h',
    'opensfm/src/third_party/akaze/lib/utils.h',
]
akaze_library = ('akaze', {
    'sources': akaze_sources,
    'depends': akaze_depends,
    'include_dirs': [akaze_include_dir],
})

# libmv
libmv_include_dir = 'opensfm/src/third_party'
libmv_sources = [
    'opensfm/src/third_party/libmv/multiview/fundamental.cc',
    'opensfm/src/third_party/libmv/multiview/projection.cc',
    'opensfm/src/third_party/libmv/multiview/five_point.cc',
    'opensfm/src/third_party/libmv/multiview/robust_five_point.cc',
    'opensfm/src/third_party/libmv/multiview/triangulation.cc',
    'opensfm/src/third_party/libmv/multiview/conditioning.cc',
    'opensfm/src/third_party/libmv/numeric/numeric.cc',
    'opensfm/src/third_party/libmv/numeric/poly.cc',
]
libmv_library = ('mv', {
    'sources': libmv_sources,
    'include_dirs': [libmv_include_dir, eigen_include_dir],
})

# VLFeat
vlfeat_include_dir = 'opensfm/src/third_party/vlfeat'
vlfeat_sources = glob.glob('opensfm/src/third_party/vlfeat/vl/*.c')
vlfeat_depends = glob.glob('opensfm/src/third_party/vlfeat/vl/*.h')
vlfeat_library = ('vl', {
    'sources': vlfeat_sources,
    'depends': vlfeat_depends,
    'macros': [('VL_DISABLE_AVX', '1')],
})

# cSfM
csfm_extension = Extension(
    'opensfm.csfm',
    sources=['opensfm/src/csfm.cc'],
    depends=['bundle.h'],
    include_dirs=[
        np.get_include(),
        eigen_include_dir,
        libmv_include_dir,
        akaze_include_dir,
        vlfeat_include_dir,
    ],
    libraries=libraries,
    library_dirs = library_dirs,
    extra_compile_args=['-std=c++11'],
)

setup(
    name='OpenSfM',
    version='0.1',
    description='A Structure from Motion library',
    url='https://github.com/mapillary/OpenSfM',
    author='Mapillary',
    license='BSD',
    packages=['opensfm'],
    libraries=[vlfeat_library, libmv_library, akaze_library],
    ext_modules=[csfm_extension],
)


