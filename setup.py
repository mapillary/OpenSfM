#!/usr/bin/env python

from distutils.core import setup
import sys
import os
import errno
import subprocess


def mkdir_p(path):
    '''Make a directory including parent directories.
    '''
    try:
        os.makedirs(path)
    except os.error as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise

print "Configuring..."
mkdir_p('cmake_build')
subprocess.Popen(['cmake','../opensfm/src'], cwd='cmake_build').wait()

print "Compiling extension..."
subprocess.Popen(['make','-j4'], cwd='cmake_build').wait()

print "Building package"
setup(
    name='OpenSfM',
    version='0.1',
    description='A Structure from Motion library',
    url='https://github.com/mapillary/OpenSfM',
    author='Mapillary',
    license='BSD',
    packages=['opensfm', 'opensfm.commands', 'opensfm.large'],
    scripts=['bin/opensfm_run_all', 'bin/opensfm'],
    package_data={
        'opensfm': ['csfm.so', 'data/sensor_data.json']
    },
)
