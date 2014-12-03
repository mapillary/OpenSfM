# -*- coding: utf-8 -*-

import os, sys
import time
import numpy as np
import json
import uuid
import cv2


def extract_feature(imagefile, config):

    image = cv2.imread(imagefile)

    # Resize the image
    sz = np.array(image.shape[0:2])
    feature_process_size = config.get('feature_process_size', -1)
    resize_ratio = feature_process_size / float(np.max(sz))
    if resize_ratio > 0 and resize_ratio < 1.0:
        image = cv2.resize(image, dsize=(0,0), fx=resize_ratio, fy=resize_ratio)
    else:
        resize_ratio = 1.0

    feature_type = config.get('feature_type','SIFT').upper()
    detector = cv2.FeatureDetector_create(feature_type)
    descriptor = cv2.DescriptorExtractor_create(feature_type)

    if feature_type == 'SIFT':
        detector.setDouble('edgeThreshold', config['sift_edge_threshold'])
        sift_peak_threshold = float(config['sift_peak_threshold'])
    elif feature_type == 'SURF':
        surf_hessian_threshold = config.get('surf_hessian_threshold',3000)
        surf_n_octaves = config.get('surf_n_octaves',4)
        surf_n_octaveLayers = config.get('surf_n_octavelayers',2)
        surf_upright = config.get('surf_upright',0)
        detector.setDouble('hessianThreshold', surf_hessian_threshold)
        detector.setDouble('nOctaves', surf_n_octaves)
        detector.setDouble('nOctaveLayers', surf_n_octaveLayers)
        detector.setInt('upright', surf_upright)

    if feature_type == 'SIFT':
        while True:
            print 'Computing sift with threshold {0}'.format(sift_peak_threshold)
            detector.setDouble("contrastThreshold", sift_peak_threshold) #default: 0.04
            points = detector.detect(image)
            print 'Found {0} points'.format(len(points))
            if len(points) < config['feature_min_frames'] and sift_peak_threshold > 0.0001:
                sift_peak_threshold = (sift_peak_threshold * 2) / 3
                print 'reducing threshold'
            else:
                print 'done'
                break
    elif feature_type == 'SURF':
        while True:
            print 'Computing surf with threshold {0}'.format(surf_hessian_threshold)
            t = time.time()
            detector.setDouble("hessianThreshold", surf_hessian_threshold) #default: 0.04
            points = detector.detect(image)
            print 'Found {0} points in {1}s'.format( len(points), time.time()-t )
            if len(points) < config['feature_min_frames'] and surf_hessian_threshold > 0.0001:
                surf_hessian_threshold = (surf_hessian_threshold * 2) / 3
                print 'reducing threshold'
            else:
                print 'done'
                break

    points, desc = descriptor.compute(image, points)
    ps = np.array([(i.pt[0]/resize_ratio, i.pt[1]/resize_ratio, i.size/resize_ratio, i.angle) for i in points])
    masks = np.array(config.get('masks',[]))
    for mask in masks:
        mask = [mask[0]*sz[0], mask[1]*sz[1], mask[2]*sz[0], mask[3]*sz[1]]
        ids  = np.invert ( (ps[:,1] > mask[0]) *
                           (ps[:,1] < mask[2]) *
                           (ps[:,0] > mask[1]) *
                           (ps[:,0] < mask[3]) )
        ps = ps[ids,:]
        desc = desc[ids,:]
    return ps, desc


def write_feature(points, descriptors,featurefile):
    np.savez(featurefile,
             points=points.astype(np.float32),
             descriptors=descriptors.astype(np.float32))

def read_feature(featurefile):
    s = np.load(featurefile)
    return s['points'], s['descriptors']


def build_flann_index(f, index_file, config):
    flann_params = dict(algorithm=2,
                        branching=config['flann_branching'],
                        terations=config['flann_iterations'])
    index = cv2.flann_Index(f, flann_params)
    index.save(index_file)


def match_lowe(index, f2, config):
    search_params = dict(checks=config['flann_checks'])
    results, dists = index.knnSearch(f2, 2, params=search_params)

    good = dists[:, 0] < config.get('lowes_ratio', 0.6) * dists[:, 1]
    matches = zip(results[good, 0], good.nonzero()[0])
    return np.array(matches, dtype=int)


def match_symmetric(fi, indexi, fj, indexj, config):
    matches_ij = [(a,b) for a,b in match_lowe(indexi, fj, config)]
    matches_ji = [(b,a) for a,b in match_lowe(indexj, fi, config)]
    matches = set(matches_ij).intersection(set(matches_ji))
    return np.array(list(matches), dtype=int)


def convert_matches_to_vector(matches):
    '''Convert Dmatch object to matrix form
    '''
    matches_vector = np.zeros((len(matches),2),dtype=np.int)
    k = 0
    for mm in matches:
        matches_vector[k,0] = mm.queryIdx
        matches_vector[k,1] = mm.trainIdx
        k = k+1
    return matches_vector


def match_lowe_bf(f1,f2,config):
    '''Bruteforce feature matching
    '''
    matcher = cv2.DescriptorMatcher_create('BruteForce')
    matches = matcher.knnMatch(f1,f2,k=2)

    good_matches = []
    for m,n in matches:
        if m.distance < 0.6*n.distance:
            good_matches.append(m)
    good_matches = convert_matches_to_vector(good_matches)
    return np.array(good_matches, dtype=int)


def robust_match(p1, p2, matches, config):
    '''Computes robust matches by estimating the Fundamental matrix via RANSAC.
    '''
    if len(matches) < 8:
        return np.array([])

    p1 = p1[matches[:, 0]][:, :2].copy()
    p2 = p2[matches[:, 1]][:, :2].copy()

    F, mask = cv2.findFundamentalMat(p1, p2, cv2.cv.CV_FM_RANSAC, config['robust_matching_threshold'], 0.99)
    inliers = mask.ravel().nonzero()

    return matches[inliers]
