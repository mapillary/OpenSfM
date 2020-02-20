import copy
import logging
import time
import cv2 as cv

from opensfm import dataset
from opensfm import exif

import cv2 as cv
logger = logging.getLogger(__name__)
logging.getLogger("exifread").setLevel(logging.WARNING)


def hard_coded_calibration(exif):
    focal = exif['focal_ratio']
    fmm35 = int(round(focal * 36.0))
    make = exif['make'].strip().lower()
    model = exif['model'].strip().lower()
    print("hello")
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
    print("hello")

def default_calibration(data):

    return {
        'focal': data.config['default_focal_prior'],
        'k1': 0.0,
        'k2': 0.0,
        'p1': 0.0,
        'p2': 0.0,
        'k3': 0.0
    }


def extract_exif():
    make,model="unknown","unknown"
    width,height=640,480
    projection_type="perspective"
    focal_ratio=0
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
    d['camera'] = "Jae Won Yang"

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

    print("calib== ",calib)
    camera = types.PerspectiveCamera()
    camera.id = metadata['camera']
    camera.width = metadata['width']
    camera.height = metadata['height']
    camera.projection_type = pt
    camera.focal = calib['focal']
    camera.k1 = calib['k1']
    camera.k2 = calib['k2']
    print()
    print(camera)
    return camera

def run(self, data):
    start=time.time()

    camera_models = {}
    print()
    #print("data.images()=== ", data.images())
    for image in data.images():
        logging.info('Extracting EXIF for {}'.format(image))
        self.d=self._extract_exif(self.image_list[image],data)
        
        data.save_exif(image, self.d)
    
    print(self.d)
    if self.d['camera'] not in camera_models:
            camera = camera_from_exif_metadata(self.d, data)
            camera_models[self.d['camera']] = camera
    
    data.meta_data_d=self.d

    data.save_camera_models(camera_models)
    end=time.time()
    print("Metadata Extracted in {}".format(end-start))


def _extract_exif(self, image ,data):
    #EXIF data in Image
    #### 여기서 metadata값 설정
    d={} 
    d=self.image_size(image,d)
    return d 
