#!/usr/bin/env python

import numpy as np
from itertools import combinations
import argparse
import dataset
import features


parser = argparse.ArgumentParser(description='Match features between all image pairs.')
parser.add_argument('dataset',
                    help='path to the dataset to be processed')
parser.add_argument('-r', '--robust', action='store_true',
                    help='compute also robust matches')
parser.add_argument('-v', '--visual', action='store_true',
                    help='plot results during the process')
args = parser.parse_args()


data = dataset.DataSet(args.dataset)
images = data.images()

for im1, im2 in combinations(images, 2):
    print 'Matching image', im1, 'with image', im2
    p1, f1 = features.read_sift(data.sift_file(im1))
    p2, f2 = features.read_sift(data.sift_file(im2))

    matches = features.match_symetric(f1, f2)
    np.savetxt(data.matches_file(im1, im2), matches, "%d")

    if args.visual:
        features.plot_matches(data.image_as_array(im1),
                              data.image_as_array(im2),
                              p1[matches[:,0]],
                              p2[matches[:,1]])
    if args.robust:
        rmatches = features.robust_match(p1, p2, matches)
        np.savetxt(data.robust_matches_file(im1, im2), rmatches, "%d")
        if args.visual:
            features.plot_matches(data.image_as_array(im1),
                                  data.image_as_array(im2),
                                  p1[rmatches[:,0]],
                                  p2[rmatches[:,1]])



