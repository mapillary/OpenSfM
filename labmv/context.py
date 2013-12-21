# -*- coding: utf-8 -*-

ROBUST_MATCHING = 'lib/build/robust_matching'
TWO_VIEW_RECONSTRUCTION = 'lib/build/two_view_reconstruction'
FLANN_PATH = 'lib/third_party/flann-1.8.4-src/src/python/build/lib'
SIFT = 'lib/third_party/vlfeat-0.9.16/bin/maci64/sift'

import sys
sys.path.append(FLANN_PATH)
import pyflann
