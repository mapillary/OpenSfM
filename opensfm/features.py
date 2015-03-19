# -*- coding: utf-8 -*-

import os, sys
import tempfile
import time
from subprocess import call
import numpy as np
import json
import uuid
import cv2

import csfm

def resized_image(image, config):
    feature_process_size = config.get('feature_process_size', -1)
    size = np.array(image.shape[0:2])
    if 0 < feature_process_size < size.max():
        new_size = size * feature_process_size / size.max()
        return cv2.resize(image, dsize=(new_size[1], new_size[0]))
    else:
        return image

def root_feature(desc, l2_normalization=False):
    if l2_normalization:
        s2 = np.linalg.norm(desc, axis=1)
        desc = (desc.T/s2).T
    s = np.sum(desc, 1)
    desc = np.sqrt(desc.T/s).T
    return desc

def root_feature_surf(desc, l2_normalization=False, partial=False):
    """
    Experimental square root mapping of surf-like feature, only work for 64-dim surf now
    """
    if desc.shape[1] == 64:
        if l2_normalization:
            s2 = np.linalg.norm(desc, axis=1)
            desc = (desc.T/s2).T
        if partial:
            ii = np.array([i for i in xrange(64) if (i%4==2 or i%4==3)])
        else:
            ii = np.arange(64)
        desc_sub = np.abs(desc[:, ii])
        desc_sub_sign = np.sign(desc[:, ii])
        # s_sub = np.sum(desc_sub, 1)  # This partial normalization gives slightly better results for AKAZE surf
        s_sub = np.sum(np.abs(desc), 1)
        desc_sub = np.sqrt(desc_sub.T/s_sub).T
        desc[:, ii] = desc_sub*desc_sub_sign
    return desc

def normalized_image_coordinates(pixel_coords, width, height):
    size = max(width, height)
    p = np.empty((len(pixel_coords), 2))
    p[:, 0] = (pixel_coords[:, 0] + 0.5 - width / 2.0) / size
    p[:, 1] = (pixel_coords[:, 1] + 0.5 - height / 2.0) / size
    return p

def denormalized_image_coordinates(norm_coords, width, height):
    size = max(width, height)
    p = np.empty((len(norm_coords), 2))
    p[:, 0] = norm_coords[:, 0] * size - 0.5 + width / 2.0
    p[:, 1] = norm_coords[:, 1] * size - 0.5 + height / 2.0
    return p

def mask_and_normalize_features(points, desc, width, height, config):
    masks = np.array(config.get('masks',[]))
    for mask in masks:
        mask = [mask[0]*height, mask[1]*width, mask[2]*height, mask[3]*width]
        ids  = np.invert ( (points[:,1] > mask[0]) *
                           (points[:,1] < mask[2]) *
                           (points[:,0] > mask[1]) *
                           (points[:,0] < mask[3]) )
        points = points[ids]
        desc = desc[ids]
    points[:, :2] = normalized_image_coordinates(points[:, :2], width, height)
    return points, desc

def extract_features_sift(image, config):
    detector = cv2.FeatureDetector_create('SIFT')
    descriptor = cv2.DescriptorExtractor_create('SIFT')
    detector.setDouble('edgeThreshold', config.get('sift_edge_threshold', 10))
    sift_peak_threshold = float(config.get('sift_peak_threshold', 0.01))
    while True:
        print 'Computing sift with threshold {0}'.format(sift_peak_threshold)
        t = time.time()
        detector.setDouble("contrastThreshold", sift_peak_threshold)
        points = detector.detect(image)
        print 'Found {0} points in {1}s'.format( len(points), time.time()-t )
        if len(points) < config.get('feature_min_frames', 0) and sift_peak_threshold > 0.0001:
            sift_peak_threshold = (sift_peak_threshold * 2) / 3
            print 'reducing threshold'
        else:
            print 'done'
            break
    points, desc = descriptor.compute(image, points)
    if config.get('feature_root', False): desc = root_feature(desc)
    points = np.array([(i.pt[0], i.pt[1], i.size, i.angle) for i in points])
    return mask_and_normalize_features(points, desc, image.shape[1], image.shape[0], config)


def extract_features_surf(image, config):
    detector = cv2.FeatureDetector_create('SURF')
    descriptor = cv2.DescriptorExtractor_create('SURF')
    surf_hessian_threshold = config.get('surf_hessian_threshold', 3000)
    detector.setDouble('hessianThreshold', surf_hessian_threshold)
    detector.setDouble('nOctaves', config.get('surf_n_octaves', 4))
    detector.setDouble('nOctaveLayers', config.get('surf_n_octavelayers', 2))
    detector.setInt('upright', config.get('surf_upright',0))
    while True:
        print 'Computing surf with threshold {0}'.format(surf_hessian_threshold)
        t = time.time()
        detector.setDouble("hessianThreshold", surf_hessian_threshold) #default: 0.04
        points = detector.detect(image)
        print 'Found {0} points in {1}s'.format( len(points), time.time()-t )
        if len(points) < config.get('feature_min_frames', 0) and surf_hessian_threshold > 0.0001:
            surf_hessian_threshold = (surf_hessian_threshold * 2) / 3
            print 'reducing threshold'
        else:
            print 'done'
            break

    points, desc = descriptor.compute(image, points)
    if config.get('feature_root', False): desc = root_feature_surf(desc, partial=True)
    points = np.array([(i.pt[0], i.pt[1], i.size, i.angle) for i in points])
    return mask_and_normalize_features(points, desc, image.shape[1], image.shape[0], config)


def akaze_descriptor_type(name):
    d = csfm.AkazeDescriptorType.__dict__
    if name in d:
        return d[name]
    else:
        print 'Wrong akaze descriptor type'
        return d['MSURF']


def extract_features_akaze(image, config):
    options = csfm.AKAZEOptions()
    options.omax = config.get('akaze_omax', 4)
    akaze_descriptor_name = config.get('akaze_descriptor', 'MSURF')
    options.descriptor = akaze_descriptor_type(akaze_descriptor_name)
    options.descriptor_size = config.get('akaze_descriptor_size', 0)
    options.descriptor_channels = config.get('akaze_descriptor_channels', 3)
    options.process_size = config.get('feature_process_size', -1)
    options.dthreshold = config.get('akaze_dthreshold', 0.001)
    options.kcontrast_percentile = config.get('akaze_kcontrast_percentile', 0.7)
    options.use_isotropic_diffusion = config.get('akaze_use_isotropic_diffusion', False)
    options.target_num_features = config.get('feature_min_frames', 0)
    options.use_adaptive_suppression = config.get('feature_use_adaptive_suppression', False)

    print 'Computing AKAZE with threshold {0}'.format(options.dthreshold)
    t = time.time()
    points, desc = csfm.akaze(image, options)
    print 'Found {0} points in {1}s'.format( len(points), time.time()-t )

    if config.get('feature_root', False):
        if akaze_descriptor_name in ["SURF_UPRIGHT", "MSURF_UPRIGHT"]:
            desc = root_feature_surf(desc, partial=True)
        elif akaze_descriptor_name in ["SURF", "MSURF"]:
            desc = root_feature_surf(desc, partial=False)
    points = points.astype(float)
    return mask_and_normalize_features(points, desc, image.shape[1], image.shape[0], config)

def extract_features_hahog(image, config):
    t = time.time()
    points, desc = csfm.hahog(image.astype(np.float32) / 255, # VlFeat expects pixel values between 0, 1
                              peak_threshold = config.get('hahog_peak_threshold', 0.003),
                              edge_threshold = config.get('hahog_edge_threshold', 10),
                              target_num_features = config.get('feature_min_frames', 0),
                              use_adaptive_suppression = config.get('feature_use_adaptive_suppression', False))

    if config.get('feature_root', False):
        desc = root_feature(desc)

    if config.get('hahog_normalize_to_uchar', False):
        scale = 512.0 / np.sqrt((desc**2).sum(axis=1))
        desc = (scale[:, np.newaxis] * desc).clip(0, 255).round()

    print 'Found {0} points in {1}s'.format( len(points), time.time()-t )
    return mask_and_normalize_features(points, desc, image.shape[1], image.shape[0], config)

def extract_features_dsift(image, config):
    t = time.time()
    points, desc = csfm.dsift(image.astype(np.float32) / 255, # VlFeat expects pixel values between 0, 1
                              step = config.get('dsift_step', 5),
                              bin_size = config.get('dsift_bin_size', 5))
    print 'Found {0} points in {1}s'.format( len(points), time.time()-t )
    if config.get('feature_root', False): desc = root_feature(desc)
    return mask_and_normalize_features(points, desc, image.shape[1], image.shape[0], config)

def extract_feature(image, config):
    if len(image.shape)==3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = resized_image(image, config)

    feature_type = config.get('feature_type','SIFT').upper()
    if feature_type == 'SIFT':
        return extract_features_sift(image, config)
    elif feature_type == 'SURF':
        return extract_features_surf(image, config)
    elif feature_type == 'AKAZE':
        return extract_features_akaze(image, config)
    elif feature_type == 'HAHOG':
        return extract_features_hahog(image, config)
    elif feature_type == 'DSIFT':
        return extract_features_dsift(image, config)
    else:
        raise ValueError('Unknown feature type (must be SURF, SIFT, AKAZE, HAHOG or DSIFT)')



def build_flann_index(features, config):
    FLANN_INDEX_LINEAR          = 0
    FLANN_INDEX_KDTREE          = 1
    FLANN_INDEX_KMEANS          = 2
    FLANN_INDEX_COMPOSITE       = 3
    FLANN_INDEX_KDTREE_SINGLE   = 4
    FLANN_INDEX_HIERARCHICAL    = 5
    FLANN_INDEX_LSH             = 6

    if features.dtype.type is np.float32:
        FLANN_INDEX_METHOD = FLANN_INDEX_KMEANS
    else:
        FLANN_INDEX_METHOD = FLANN_INDEX_LSH

    flann_params = dict(algorithm=FLANN_INDEX_METHOD,
                        branching=config.get('flann_branching', 16),
                        iterations=config.get('flann_iterations', 20))
    index = cv2.flann_Index(features, flann_params)
    return index
