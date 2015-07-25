#!/usr/bin/env python
import os.path, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import json
import datetime
import exifread
import numpy as np
from cv2 import imread

from opensfm.sensors import sensor_data


def eval_frac(value):
    return float(value.num) / float(value.den)

def gps_to_decimal(values, reference):
    sign = 1 if reference in 'NE' else -1
    degrees = eval_frac(values[0])
    minutes = eval_frac(values[1])
    seconds = eval_frac(values[2])
    return sign * (degrees + minutes / 60 + seconds / 3600)

def get_float_tag(tags, key):
    if key in tags:
        return float(tags[key].values[0])
    else:
        return None

def get_frac_tag(tags, key):
    if key in tags:
        return eval_frac(tags[key].values[0])
    else:
        return None

def compute_focal(focal_35, focal, sensor_width, sensor_string):
    if focal_35 > 0:
        focal_ratio = focal_35 / 36.0 # 35mm film produces 36x24mm pictures.
    else:
        if not sensor_width:
            sensor_width = sensor_data.get(sensor_string, None)
        if sensor_width and focal:
            focal_ratio = focal / sensor_width
            focal_35 = 36.0 * focal_ratio
        else:
            focal_35 = 0
            focal_ratio = 0
    return focal_35, focal_ratio

def get_distortion(make, model, fmm35):
    if 'gopro' in make.lower():
        if fmm35==20:
            # GoPro Hero 3, 7MP medium
            ## calibration
            d = np.array([-0.37, 0.28, 0, 0, 0])
        elif fmm35==15:
            # GoPro Hero 3, 7MP wide
            # calibration
            d = np.array([-0.32, 0.24, 0, 0, 0])
        elif fmm35==23:
            # GoPro Hero 2, 5MP medium
            d = np.array([-0.38, 0.24, 0, 0, 0])
        elif fmm35==16:
            # GoPro Hero 2, 5MP wide
            d = np.array([-0.39, 0.22, 0, 0, 0])
        else:
            raise ValueError("Unsupported f value.")

        return list(d)
    else:
        return [0., 0., 0., 0., 0.]


def sensor_string(make, model):
    if make != 'unknown':
        model = model.replace(make, '') # remove possible duplicate 'make' information in 'model' for better matching
    return (make.strip() + ' ' + model.strip()).lower()


def extract_exif_from_file(fileobj):
    if isinstance(fileobj, (str, unicode)):
        with open(fileobj) as f:
            exif_data = EXIF(f)
    else:
        exif_data = EXIF(fileobj)

    d = exif_data.extract_exif()
    return d

class EXIF:

    def __init__(self, fileobj):
        self.tags = exifread.process_file(fileobj, details=False)

    def extract_image_size(self):
        # Image Width and Image Height
        if 'EXIF ExifImageWidth' in self.tags and 'EXIF ExifImageLength' in self.tags:
            width, height = (int(self.tags['EXIF ExifImageWidth'].values[0]),
                            int(self.tags['EXIF ExifImageLength'].values[0]) )
        else:
            width, height = -1, -1
        return width, height

    def extract_make(self):
        # Camera make and model
        if 'EXIF LensMake' in self.tags:
            make = self.tags['EXIF LensMake'].values
        elif 'Image Make' in self.tags:
            make = self.tags['Image Make'].values
        else:
            make = 'unknown'
        try:
            return make.decode('utf-8')
        except UnicodeDecodeError:
            return 'unknown'

    def extract_model(self):
        if 'EXIF LensModel' in self.tags:
            model = self.tags['EXIF LensModel'].values
        elif 'Image Model' in self.tags:
            model = self.tags['Image Model'].values
        else:
            model = 'unknown'
        try:
            return model.decode('utf-8')
        except UnicodeDecodeError:
            return 'unknown'

    def extract_focal(self):
        make, model = self.extract_make(), self.extract_model()
        focal_35, focal_ratio = compute_focal(
            get_float_tag(self.tags, 'EXIF FocalLengthIn35mmFilm'),
            get_frac_tag(self.tags, 'EXIF FocalLength'),
            get_frac_tag(self.tags, 'EXIF CCD width'),
            sensor_string(make, model))
        return focal_35, focal_ratio

    def extract_orientation(self):
        if 'Image Orientation' in self.tags:
            orientation = self.tags.get('Image Orientation').values[0]
        else:
            orientation = 1
        return orientation

    def extract_distortion(self):
        make, model = self.extract_make(), self.extract_model()
        fmm35, fratio = self.extract_focal()
        distortion = get_distortion(make, model, fmm35)
        return distortion[0], distortion[1]

    def extract_lon_lat(self):
        if 'GPS GPSLatitude' in self.tags:
            lat = gps_to_decimal(self.tags['GPS GPSLatitude'].values,
                                 self.tags['GPS GPSLatitudeRef'].values)
            lon = gps_to_decimal(self.tags['GPS GPSLongitude'].values,
                                 self.tags['GPS GPSLongitudeRef'].values)
        else:
            lon, lat = None, None
        return lon, lat

    def extract_altitude(self):
        if 'GPS GPSAltitude' in self.tags:
            altitude = eval_frac(self.tags['GPS GPSAltitude'].values[0])
        else:
            altitude = None
        return altitude

    def extract_dop(self):
        if 'GPS GPSDOP' in self.tags:
            dop = eval_frac(self.tags['GPS GPSDOP'].values[0])
        else:
            dop = None
        return dop

    def extract_geo(self):
        altitude = self.extract_altitude()
        dop = self.extract_dop()
        lon, lat = self.extract_lon_lat()
        d = {}

        if lon is not None and lat is not None:
            d['latitude'] = lat
            d['longitude'] = lon
        if altitude is not None:
            d['altitude'] = altitude
        if dop is not None:
            d['dop'] = dop
        return d

    def extract_capture_time(self):
        time_strings = ["EXIF DateTimeOriginal",
                        "EXIF DateTimeDigitized",
                        "Image DateTime"]
        for ts in time_strings:
            if ts in self.tags:
                s = str(self.tags[ts].values)
                d = datetime.datetime.strptime(s, '%Y:%m:%d %H:%M:%S')
                timestamp = (d - datetime.datetime(1970,1,1)).total_seconds()   # Assuming d is in UTC
                return timestamp
        return 0.0


    def extract_exif(self):
        width, height = self.extract_image_size()
        focal_35, focal_ratio = self.extract_focal()
        make, model = self.extract_make(), self.extract_model()
        orientation = self.extract_orientation()
        geo = self.extract_geo()
        distortion = self.extract_distortion()
        capture_time = self.extract_capture_time()
        d = {
                'width': width,
                'height': height,
                'focal_prior': focal_ratio,
                'camera': sensor_string(make, model),
                'orientation': orientation,
                'k1': distortion[0],
                'k2': distortion[1],
                'make': make,
                'model': model,
                'capture_time': capture_time
            }
        # GPS
        d['gps'] = geo
        return d





