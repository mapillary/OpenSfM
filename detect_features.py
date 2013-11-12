#!/usr/bin/env python

import os, sys
import dataset
import features

def usage():
    print 'USAGE: %s data_set_path' % sys.argv[0]
    sys.exit(0)
    

if len(sys.argv) > 1:
    path = sys.argv[1]
else:
    usage()

data = dataset.DataSet(path)
images = data.images()

for image in images:
    print 'Extracting SIFT features for image', image
    features.extract_sift(data.image_file(image), data.sift_file(image))
