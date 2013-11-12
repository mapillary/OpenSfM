#!/usr/bin/env python

import numpy as np
from itertools import combinations
import argparse
import dataset
import features


parser = argparse.ArgumentParser(description='Compute robust matches by estimating the Fundamental matrix')
parser.add_argument('dataset',
                    help='path to the dataset to be processed')
parser.add_argument('-v', '--visual', action='store_true',
                    help='plot results during the process')
args = parser.parse_args()


data = dataset.DataSet(args.dataset)
images = data.images()

for im1, im2 in combinations(images, 2):
    print 'Matching image', im1, 'with image', im2
    p1, f1 = features.read_sift(data.sift_file(im1))
    p2, f2 = features.read_sift(data.sift_file(im2))

    matches = np.genfromtxt(data.matches_file(im1, im2), dtype=int)

    rmatches = features.robust_match(p1, p2, matches)

    if len(rmatches) > 20:
        with open(data.robust_matches_file(im1, im2), 'w') as fout:
            lines = [str(a) + ' ' + str(b) for a,b in rmatches]
            fout.write('\n'.join(lines))

        if args.visual:
            features.plot_matches(data.image_as_array(im1),
                                  data.image_as_array(im2),
                                  p1[rmatches[:,0]],
                                  p2[rmatches[:,1]])


