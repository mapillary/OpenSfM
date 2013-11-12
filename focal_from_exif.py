#!/usr/bin/env python

import os, sys
import dataset
import json
from subprocess import check_output

JHEAD = '/usr/local/bin/jhead'

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
    print 'Extracting focal lengths for image', image

    exif = check_output([JHEAD, data.image_file(image)])
    size = exif.split('Resolution   :')[1].split('\n')[0].split('x')
    width, height = [int(i) for i in size]
    focal_35 = float(exif.split('(35mm equivalent:')[1].split('mm')[0])
    focal_ratio = focal_35 / 36.0 # 35mm film produces 36x24mm pictures.

    d = {
            'width': width,
            'height': height,
            'focal_ratio': focal_ratio,
            'focal_35mm_equiv': focal_35,
        }
    print d
    with open(data.exif_file(image), 'w') as fout:
        fout.write(json.dumps(d, indent=4))