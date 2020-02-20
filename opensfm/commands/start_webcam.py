import cv2 as cv
import os
from os.path import abspath, join, dirname
import copy
import logging
import argparse
import time
import sys
import numpy as np
import errno
import io
import json

#from opensfm.commands.extract_metadata import *
from opensfm import dataset
from opensfm import log
from opensfm import io
from opensfm import exif
from opensfm import types
from opensfm import config


from PIL import Image

logger = logging.getLogger(__name__)
logging.getLogger("Starting Webcam!!").setLevel(logging.WARNING)


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



class DataSet(object):
	"""Accessors to the main input and output data.

    Data include input images, masks, and segmentation as well
    temporary data such as features and matches and the final
    reconstructions.

    All data should be stored inside a variable to use RAM.
    Not in the folder.

    It is possible to store data remotely or in different formats
    by subclassing this class and overloading its methods.
    """
	def __init__(self, data_path, image_list):
		self.data_path=data_path
		self.image_list=image_list
		self._load_config()
		self.meta_data_d={}

	def _load_config(self):
		config_file = os.path.join(self.data_path, 'config.yaml')
		self.config = config.load_config(config_file)
	def show(self, image):
		cv.imshow('test',image)
		cv.waitKey(0)
		cv.destroyAllWindows()
	def add(self, x, y):
		print(x+y+pi)
	def images(self):
		"""List of all images in the dataset""" 
		return self.image_list.keys()
	def image_size(self,image,d):
		d=extract_exif()
		d['height']=image.shape[0]
		d['width']=image.shape[1]

		return d
	def _exif_file(self, image):
	    """
	    Return path of exif information for given image
	    :param image: Image name, with extension (i.e. 123.jpg)
	    """      
	    return os.path.join(self._exif_path(), image + '.exif')
	def _exif_path(self):
		"""Return path of extracted exif directory"""
		return os.path.join(self.data_path, 'exif')

	def save_exif(self, image, data):
		try:
			os.makedirs(self._exif_path())
		except os.error as exc:
		    if exc.errno != errno.EEXIST or not os.path.isdir(self._exif_path()):
		        raise
		with io.open_wt(self._exif_file(image)) as fout:
		    io.json_dump(data, fout)
		# with io.open_wt(self._exif_file(image)) as fout:
		#     io.json_dump(data, fout)
	def save_camera_models(self, camera_models):
	    """Save camera models data"""
	    with io.open_wt(self._camera_models_file()) as fout:
	        obj = io.cameras_to_json(camera_models)
	        io.json_dump(obj, fout)

	def _camera_models_file(self):
	    """Return path of camera model file"""
	    return os.path.join(self.data_path, 'camera_models.json')
	


class extract_metadata(DataSet):
	
	def __init__(self, data_path, image_list):
		super().__init__(data_path,image_list)
		

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

class detect_features(DataSet):

	def __init__(self,data):
		self.image_list=data.image_list
		self.data_path=data.data_path
		self.meta_data_d=data.meta_data_d
	def run(self,data):
		print(self.image_list.keys())
		#self.show(self.image_list['1.jpg'])
		print(self.meta_data_d)



class Command:
	name = 'start_webcam'
	help = "Starting webcam for image capture"

	def add_arguments(self, parser):
		parser.add_argument('dataset', help='dataset to process')

	def run(self, args):
		self.save_webcamImage(args)
		print(self.image_list.keys())
		
		#*****
		data=DataSet(args.dataset,self.image_list)
		#******

		meta_data=extract_metadata(data.data_path,data.image_list)
		meta_data.run(data)# data객체 전달 (이미지포함됨) 
		detect_data=detect_features(data)
		detect_data.run(data)


	def save_webcamImage(self, args):
		cap=cv.VideoCapture(0)
		i=0
		count=1
		self.image_list={}
		if(cap.isOpened()==False):
		    print("Unable to open the webcam!!")

		while(cap.isOpened()):
		    ret, frame=cap.read()
		    if ret==False:
		        break
		   
		    cv.imshow('camera',frame)
		    #print(type(frame))#== numpy.ndarray'	    
		    if i%2==0:
		    	img_name = "{}.jpg".format(count)
		    	#self.image_list.append(frame)
		    	self.image_list.update({img_name:frame})
		    	logging.info('Capturing Images for {}'.format(img_name))
		    	if count%10 ==0:
		    		#print('data_path==',args)	
		    		#Namespace(command='start_webcam', dataset='data/maintest')
		    		#self.webcam_data=DataSet(args.dataset,self.image_list)
		    		#self.webcam_data.image_list=self.image_list
		    		break
		    		#run(data_path, image_list)
		    	#cv.imwrite(os.path.join(data_path, img_name),frame)
		    	count+=1

		    if cv.waitKey(1) & 0xFF==27:
		        break
		    i=i+1
		cap.release()
		cv.destroyAllWindows()

# log.setup()
# args=sys.argv[1]

# save_webcamImage(args)
# print(args)

