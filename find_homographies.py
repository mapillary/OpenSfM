#!/usr/bin/env python

import numpy as np
import pylab as pl
import sys
import argparse
import json
import dataset
import features
import networkx as nx
from networkx.algorithms import bipartite

sys.path.insert(0, '../PCV')

from PCV.geometry import homography



parser = argparse.ArgumentParser(description=
    'Compute an homography between every image pair in the tracks graph')
parser.add_argument('dataset',
                    help='path to the dataset to be processed')
parser.add_argument('-v', '--visual', action='store_true',
                    help='plot results during the process')
args = parser.parse_args()


data = dataset.DataSet(args.dataset)
images = data.images()


# Build a biparted graph connecting images and tracks.
g = data.tracks_graph()

# Get the image connectivity graph.
track_nodes, image_nodes = bipartite.sets(g)
image_graph = bipartite.weighted_projected_graph(g, image_nodes)

result = []

# Iterate over neighboring images.
for im1, im2 in image_graph.edges():
    print 'Matching image', im1, 'with image', im2
    d1 = data.exif_data(im1)
    d2 = data.exif_data(im2)

    t1 = g[im1]
    t2 = g[im2]
    p1 = []
    p2 = []
    for track in t1:
        if track in t2:
            p1.append(t1[track]['feature'])
            p2.append(t2[track]['feature'])
    p1 = np.array(p1)
    p2 = np.array(p2)
    if len(p1) > 20:
        h1 = np.ones((3, len(p1)))
        h2 = np.ones((3, len(p1)))
        h1[:2] = p1.T
        h2[:2] = p2.T

        model = homography.RansacModel()
        try:
            H, inliers = homography.H_from_ransac(h1, h2, model, maxiter=1000, match_theshold=10)
        except:
            continue

        if len(inliers) > 20:
            result.append({
                'image1': im1,
                'image2': im2,
                'H': list(H.flat),
                })
            print result[-1]

            if args.visual:
                features.plot_matches(data.image_as_array(im1),
                                      data.image_as_array(im2),
                                      p1[inliers],
                                      p2[inliers])

                Hh1 = np.dot(H, h1)
                Hh1 = Hh1 / Hh1[2]

                pl.imshow(data.image_as_array(im2))
                pl.plot(h2[0], h2[1],'ob')
                pl.plot(Hh1[0], Hh1[1],'or')
                pl.show()



with open('homographies.json', 'w') as fout:
    json.dump(result, fout, indent=4)

