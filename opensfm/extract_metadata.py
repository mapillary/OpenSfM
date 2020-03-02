import copy
import logging
import time
import cv2 as cv

from opensfm import dataset
from opensfm import exif
from opensfm import dataset
from opensfm import log
from opensfm import io
from opensfm import exif
from opensfm import types
from opensfm import config

import cv2 as cv
logger = logging.getLogger(__name__)
logging.getLogger("exifread").setLevel(logging.WARNING)


def hard_coded_calibration(exif):
    focal = exif['focal_ratio']
    fmm35 = int(round(focal * 36.0))
    make = exif['make'].strip().lower()
    model = exif['model'].strip().lower()
    if 'gopro' in make:
        if fmm35 == 20:
            # GoPro Hero 3, 7MP medium
            return {'focal': focal, 'k1': -0.37, 'k2': 0.28}
        elif fmm35 == 15:
            # GoPro Hero 3, 7MP wide
            # "v2 gopro hero3+ black edition 3000 2250 perspective 0.4166"
            return {'focal': 0.466, 'k1': -0.195, 'k2': 0.030}
        elif fmm35 == 23:
            # GoPro Hero 2, 5MP medium
            return {'focal': focal, 'k1': -0.38, 'k2': 0.24}
        elif fmm35 == 16:
            # GoPro Hero 2, 5MP wide
            return {'focal': focal, 'k1': -0.39, 'k2': 0.22}
    elif 'bullet5s' in make:
        return {'focal': 0.57, 'k1': -0.30, 'k2': 0.06}
    elif 'garmin' == make:
        if 'virb' == model:
            # "v2 garmin virb 4608 3456 perspective 0"
            return {'focal': 0.5, 'k1': -0.08, 'k2': 0.005}
        elif 'virbxe' == model:
            # "v2 garmin virbxe 3477 1950 perspective 0.3888"
            # "v2 garmin virbxe 1600 1200 perspective 0.3888"
            # "v2 garmin virbxe 4000 3000 perspective 0.3888"
            # Calibration when using camera's undistortion
            return {'focal': 0.466, 'k1': -0.08, 'k2': 0.0}
            # Calibration when not using camera's undistortion
            # return {'focal': 0.466, 'k1': -0.195, 'k2'; 0.030}
    elif 'drift' == make:
        if 'ghost s' == model:
            return {'focal': 0.47, 'k1': -0.22, 'k2': 0.03}
    elif 'xiaoyi' in make:
        return {'focal': 0.5, 'k1': -0.19, 'k2': 0.028}
    elif 'geo' == make and 'frames' == model:
        return {'focal': 0.5, 'k1': -0.24, 'k2': 0.04}
    elif 'sony' == make:
        if 'hdr-as200v' == model:
            return {'focal': 0.55, 'k1': -0.30, 'k2': 0.08}
        elif 'hdr-as300' in model:
            return {"focal":  0.3958, "k1": -0.1496, "k2": 0.0201}


def focal_ratio_calibration(exif):

    if exif.get('focal_ratio'):
        return {
            'focal': exif['focal_ratio'],
            'k1': 0.0,
            'k2': 0.0,
            'p1': 0.0,
            'p2': 0.0,
            'k3': 0.0
        }

def default_calibration(data):

    return {
        'focal': data.config['default_focal_prior'],
        'k1': 0.0,
        'k2': 0.0,
        'p1': 0.0,
        'p2': 0.0,
        'k3': 0.0
    }


def extract_exif(image, data):
    make,model="unknown","unknown"
    width,height=data.image_size(image)
    projection_type="perspective"
    focal_ratio=0.85
    orientation=1
    d = {
            'make': make,
            'model': model,
            'width': width,
            'height': height,
            'projection_type': projection_type,
            'focal_ratio': focal_ratio,
            #'orientation': orientation,
            #'capture_time': capture_time,
            #'gps': geo
        }
    d['camera'] = "v2 unknown unknown {} {} perspective 0".format(width,height)

    return d

def camera_from_exif_metadata(metadata, data):
    '''
    Create a camera object from exif metadata
    '''
    pt = metadata.get('projection_type', 'perspective').lower()
    if pt == 'perspective':
        calib = (hard_coded_calibration(metadata)
                 or focal_ratio_calibration(metadata)
                 or default_calibration(data)
                 )

    #print("calib== ",calib['focal'])
    camera = types.PerspectiveCamera()
    camera.id = metadata['camera']
    camera.width = metadata['width']
    camera.height = metadata['height']
    camera.projection_type = pt
    camera.focal = calib['focal']
    camera.k1 = calib['k1']
    camera.k2 = calib['k2']
    return camera,calib

def run(data):
    start=time.time()

    camera_models = {}
    print()
    #print("data.images()=== ", data.images())
    for image in data.images():
        logging.info('Extracting EXIF for {}'.format(image))
        d=_extract_exif(data.image_list[image],data)
        #data.save_exif(image, d)

    if d['camera'] not in camera_models:
        camera,calib = camera_from_exif_metadata(d, data)
        camera_models[d['camera']] = camera
    print(camera_models)
    data.meta_data_d=d
    
    data.camera_models=camera_models
    #data.save_camera_models(camera_models)
    end=time.time()
    
    print("Metadata Extracted in {}".format(end-start))
    d.update({'focal':calib['focal']})
    with open(data.profile_log(), 'a') as fout:
            fout.write('extract_metadata: {0}\n'.format(end - start))
    
    return d


def _extract_exif(image ,data):
    #EXIF data in Image
    #### 여기서 metadata값 설정
    d=extract_exif(image,data)
    
    return d 
