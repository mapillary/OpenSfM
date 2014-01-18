# -*- coding: utf-8 -*-

from PIL import Image
import os, sys
from subprocess import call, Popen, PIPE
import numpy as np
import pylab as pl
import context
import json


def extract_sift(image, sift, params=["--edge-thresh=10", "--peak-thresh=5"]):
    '''Extracts SIFT features of image and save them in sift
    '''
    Image.open(image).convert('L').save('tmp.pgm')
    call([context.SIFT, "tmp.pgm", "--output=%s" % sift] + params)
    os.remove('tmp.pgm')


def read_sift(siftfile):
    s = np.loadtxt(siftfile)
    return s[:,:4], s[:,4:]


def match_lowe(f1, f2):
    results, dists = context.pyflann.FLANN().nn(f1, f2, 2,
        checks=128, algorithm='kmeans', branching=64, iterations=5)

    good = dists[:, 0] < 0.6 * dists[:, 1]
    matches = zip(results[good, 0], good.nonzero()[0])
    return np.array(matches, dtype=int)


def match_symetric(fi, fj):
    matches_ij = [(a,b) for a,b in match_lowe(fi, fj)]
    matches_ji = [(b,a) for a,b in match_lowe(fj, fi)]
    matches = set(matches_ij).intersection(set(matches_ji))
    return np.array(list(matches), dtype=int)


def robust_match(p1, p2, matches):
    '''Computes robust matches by estimating the Fundamental matrix via RANSAC.
    '''
    p1 = p1[matches[:, 0]][:, :2]
    p2 = p2[matches[:, 1]][:, :2]
    s = ''
    for l in np.hstack((p1, p2)):
        s += ' '.join(str(i) for i in l) + '\n'

    p = Popen([context.ROBUST_MATCHING, '-threshold', '0.1'],
              stdout=PIPE, stdin=PIPE, stderr=PIPE)
    res = p.communicate(input=s)[0]
    inliers = [int(i) for i in res.split()]
    return matches[inliers]


def two_view_reconstruction(p1, p2, d1, d2):
    '''Computes a two view reconstruction from a set of matches.
    '''
    s = ''
    for l in np.hstack((p1, p2)):
        s += ' '.join(str(i) for i in l) + '\n'

    params = [context.TWO_VIEW_RECONSTRUCTION,
              '-threshold', '5',
              '-focal1', d1['focal_ratio'] * d1['width'],
              '-width1', d1['width'],
              '-height1', d1['height'],
              '-focal2', d2['focal_ratio'] * d2['width'],
              '-width2', d2['width'],
              '-height2', d2['height']]
    params = map(str, params)

    p = Popen(params, stdout=PIPE, stdin=PIPE, stderr=PIPE)
    res = p.communicate(input=s)[0]

    res = res.split(None, 9 + 3)
    Rt_res = map(float, res[:-1])
    inliers_res = res[-1]
    R = np.array(Rt_res[:9]).reshape(3,3)
    t = np.array(Rt_res[9:])

    inliers = []
    Xs = []
    for line in inliers_res.splitlines():
        words = line.split()
        inliers.append(int(words[0]))
        Xs.append(map(float, words[1:]))

    return R, t, inliers, Xs


def bundle(tracks_file, reconstruction):
    '''Extracts SIFT features of image and save them in sift
    '''
    source = "/tmp/bundle_source.json"
    dest = "/tmp/bundle_dest.json"

    with open(source, 'w') as fout:
        fout.write(json.dumps(reconstruction, indent=4))

    call([context.BUNDLE, tracks_file, source, dest])

    with open(dest) as fin:
        return json.load(fin)



def plot_features(im, p):
    pl.imshow(im)
    pl.plot(p[:,0],p[:,1],'ob')
    pl.show()


def plot_matches(im1, im2, p1, p2):
    h1, w1, c = im1.shape
    h2, w2, c = im1.shape
    image = np.zeros((max(h1,h2), w1+w2, 3), dtype=im1.dtype)
    image[0:h1, 0:w1, :] = im1
    image[0:h2, w1:(w1+w2), :] = im2
    
    pl.imshow(image)
    for a1, a2 in zip(p1,p2):
        pl.plot([a1[0], a2[0] + w1], [a1[1], a2[1]], 'c')

    pl.plot(p1[:,0], p1[:,1], 'ob')
    pl.plot(p2[:,0] + w1, p2[:,1], 'ob')
    pl.show()


