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

from opensfm import extract_metadata
from opensfm import detect_features

from opensfm import dataset
from opensfm import log
from opensfm import io
from opensfm import exif
from opensfm import types
from opensfm import config
from opensfm import features


from PIL import Image

logger = logging.getLogger(__name__)
logging.getLogger("Starting Webcam!!").setLevel(logging.WARNING)

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
		self._load_mask_list()
		self.meta_data_d={}
		self.feature_points={}
		self.feature_colors={}
		self.feature_descriptors={}
		self.feature_of_images={}
		self.feature_report={}

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

	def _image_file(self, image):
	    """Path to the image file."""
	    return self.image_list[image]

	def image_size(self,image,d):
		d=extract_metadata.extract_exif()
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

	def _load_mask_list(self):
	    """Load mask list from mask_list.txt or list masks/ folder."""
	    mask_list_file = os.path.join(self.data_path, 'mask_list.txt')
	    if os.path.isfile(mask_list_file):
	        with io.open_rt(mask_list_file) as fin:
	            lines = fin.read().splitlines()
	        self._set_mask_list(lines)
	    else:
	        self._set_mask_path(os.path.join(self.data_path, 'masks'))
	
	def _set_mask_path(self, path):
	    """Set mask path and find all masks in there"""
	    self.mask_files = {}
	    for image in self.images():
	        filepath = os.path.join(path, image + '.png')
	        print(filepath)# data/maintest/masks/1.jpg.png
	        if os.path.isfile(filepath):
	            self.mask_files[image] = filepath            

	def features_exist(self, image):
	    return os.path.isfile(self._feature_file(image)) or\
	        os.path.isfile(self._feature_file_legacy(image))

	def feature_type(self):
	    """Return the type of local features (e.g. AKAZE, SURF, SIFT)"""
	    feature_name = self.config['feature_type'].lower()
	    if self.config['feature_root']:
	        feature_name = 'root_' + feature_name
	    return feature_name
	
	def load_image(self, image, unchanged=False, anydepth=False):
		"""Load image pixels as numpy array.

		The array is 3D, indexed by y-coord, x-coord, channel.
		The channels are in RGB order.
		"""
		a=io.imread(self._image_file(image), unchanged=unchanged, anydepth=anydepth)
		return a

	def load_mask(self, image):
	    """Load image mask if it exists, otherwise return None."""
	    if image in self.mask_files: ## 여기 안들어감 
	        mask_path = self.mask_files[image]
	        mask = io.imread(mask_path, grayscale=True)
	       
	        if mask is None:
	            raise IOError("Unable to load mask for image {} "
	                          "from file {}".format(image, mask_path))
	    else:
	        mask = None
	    return mask

	def load_segmentation_mask(self, image):
	    """Build a mask from segmentation ignore values.

	    The mask is non-zero only for pixels with segmentation
	    labels not in segmentation_ignore_values.
	    """
	    ignore_values = self.segmentation_ignore_values(image)
	    if not ignore_values:	
	        return None
	    segmentation = self.load_segmentation(image)
	    if segmentation is None:
	        return None

	    return self._mask_from_segmentation(segmentation, ignore_values)

	def _combine_masks(self, mask, smask):
	    if mask is None:
	        if smask is None:
	            return None
	        else:
	            return smask
	    else:
	        if smask is None:
	            return mask
	        else:
	            return mask & smask

	def segmentation_ignore_values(self, image):
	    """List of label values to ignore.

	    Pixels with this labels values will be masked out and won't be
	    processed when extracting features or computing depthmaps.
	    """
	    return self.config.get('segmentation_ignore_values', [])

	def load_combined_mask(self, image):
	    """Combine binary mask with segmentation mask.
	    Return a mask that is non-zero only where the binary
	    mask and the segmentation mask are non-zero.
	    """
	    mask = self.load_mask(image) # 필요없다 None값
	    smask = self.load_segmentation_mask(image) # 필요없다 None 값 
	    return self._combine_masks(mask, smask)
	
	def load_features_mask(self, image, points):
	    """Load a feature-wise mask.

	    This is a binary array true for features that lie inside the
	    combined mask.
	    The array is all true when there's no mask.
	    """
	    if points is None or len(points) == 0:
	        return np.array([], dtype=bool)

	    mask_image = self.load_combined_mask(image)
	    
	    if mask_image is None:
	        logger.debug('No segmentation for {}, no features masked.'.format(image))
	        return np.ones((points.shape[0],), dtype=bool)
   
   		#detect features명령어에선 여기서 종료함 *************
	    exif = self.load_exif(image)
	    width = exif["width"]
	    height = exif["height"]
	    orientation = exif["orientation"]

	    new_height, new_width = mask_image.shape
	    ps = upright.opensfm_to_upright(
	        points[:, :2], width, height, orientation,
	        new_width=new_width, new_height=new_height).astype(int)
	    mask = mask_image[ps[:, 1], ps[:, 0]]

	    n_removed = np.sum(mask == 0)
	    logger.debug('Masking {} / {} ({:.2f}) features for {}'.format(
	        n_removed, len(mask), n_removed / len(mask), image))

	    return np.array(mask, dtype=bool)
	
	def _feature_file(self, image):
	    """
	    Return path of feature file for specified image
	    :param image: Image name, with extension (i.e. 123.jpg)
	    """
	    return os.path.join(self._feature_path(), image + '.features.npz')
	
	def _feature_path(self):
	    """Return path of feature descriptors and FLANN indices directory"""
	    return os.path.join(self.data_path, "features")

	def _save_features(self, filepath, points, descriptors, colors=None):
	    io.mkdir_p(self._feature_path())
	    features.save_features(filepath, points, descriptors, colors, self.config)

	def save_features(self, image, points, descriptors, colors):
		self._save_features(self._feature_file(image), points, descriptors, colors)

	def save_total_features(self,feature_of_images):

		self.feature_of_images=feature_of_images
	def save_report_of_features(self,image,report):
		self.feature_report.update({image:report})

	def _report_path(self):
	    return os.path.join(self.data_path, 'reports')

	def save_report(self, report_str, path):
		"""Save report string to a file."""
		filepath = os.path.join(self._report_path(), path)
		io.mkdir_p(os.path.dirname(filepath))
		with io.open_wt(filepath) as fout:
		    return fout.write(report_str)
	
	def load_report(self, path):
	    """Load a report file as a string."""
	    with io.open_rt(os.path.join(self._report_path(), path)) as fin:
	        return fin.read()

	def profile_log(self):
	    #"Filename where to write timings."
	    return os.path.join(self.data_path, 'profile.log')



class SLAM():
	def __init__(self,data):
		self.data=data
		self.meta_data={}
		
		
	def Metadata(self):
		self.meta_data=extract_metadata.run(self.data)
		print("meta_data==", self.meta_data)

	def detect_Features(self):
		DF=detect_features.detecting_features()
		DF.run(self.data)

		#self.data.feature_of_images 저장완료
		#self.data.feature_report 저장완료 


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
		slam=SLAM(data)
		slam.Metadata()
		slam.detect_Features()

		
	

		print("yjw")


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
		    if i%1==0:
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

