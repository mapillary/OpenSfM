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
import gzip
import pickle
import six
import networkx as nx

from opensfm import dataset
from opensfm import extract_metadata
from opensfm import detect_features
from opensfm import match_features
from opensfm import create_tracks
from opensfm import reconstruct
from opensfm import mesh_data
from opensfm import undistort
from opensfm import undistorted_dataset
from opensfm import compute_depthmaps


from opensfm import log
from opensfm import io
from opensfm import exif
from opensfm import types
from opensfm import config
from opensfm import features
from opensfm import tracking
from opensfm import io
from opensfm.context import current_memory_usage, memory_available



from PIL import Image

logger = logging.getLogger(__name__)
logging.getLogger("Starting Webcam!!").setLevel(logging.WARNING)

class SLAM():
	def __init__(self,data):
		self.data=data
		
	def Metadata(self):
		self.meta_data=extract_metadata.run(self.data)
		print("meta_data==", self.meta_data)

	def detect_Features(self):
		DF=detect_features.detecting_features()
		DF.run(self.data)
		#self.data.feature_of_images 저장완료
		#self.data.feature_report 저장완료 

	def match_Features(self):
		MF=match_features.run(self.data)

	def create_tracks(self):
		create_tracks.run(self.data)

	def reconstruct(self):
		reconstruct.run(self.data)

	# def mesh(self):
	# 	mesh_data.run(self.data)
	
	def visualize_slam(self):
		ply= io.reconstruction_to_ply(self.data.reconstructions_as_json)
		with io.open_wt(self.data._depthmap_path() + '/SLAM.ply') as fout:
		        fout.write(ply)

	def undistorting(self):
		undistort.run(self.data)
	
	def compute_depthmaps(self):
		compute_depthmaps.run(self.data)


class Command:
	name = 'slam'
	help = "Starting webcam for image capture"

	def add_arguments(self, parser):
		parser.add_argument('dataset', help='dataset to process')
		parser.add_argument('webcam', help='webcam status 0:off  1: on')

	def run(self, args):
		print(args)
		self.data_path=args.dataset
		if(args.webcam=='0'):#webcam off 
			self.load_image_list(self.data_path)
		else:	#webcam on 
			self.save_webcamImage(self.data_path)

		#print(self.image_list.keys())
		print(self.image_list.keys())
		
		start=time.time()
		data=dataset.DataSet(args.dataset,self.image_list)
		
		print('available memory== ', memory_available())
		print("current memory usage==", current_memory_usage())

		slam=SLAM(data)
		slam.Metadata()
		slam.detect_Features()
		slam.match_Features()
		slam.create_tracks()
		slam.reconstruct()
		
		slam.visualize_slam()
		end=time.time()
		recon_time=end-start
		print("Reconstruction Time == {:.0f}m {:.0f}s".format(recon_time//60, recon_time%60))
		
		# slam.undistorting()
		# slam.compute_depthmaps()
		
		# end=time.time()
		# dense_time=end-start
		# print("Computing depth Time == {:.0f}m {:.0f}s".format(dense_time//60, dense_time%60))

		end=time.time()
		slam_time=end-start
		print("Total SLAM Time == {:.0f}m {:.0f}s".format(slam_time//60, slam_time%60))	

	def load_image_list(self, data_path):
		print(data_path)
		self._set_image_path(os.path.join(data_path, 'images'))

	def _is_image_file(self, filename):
		extensions = {'jpg', 'jpeg', 'png', 'tif', 'tiff', 'pgm', 'pnm', 'gif'}
		return filename.split('.')[-1].lower() in extensions

	def _set_image_path(self,data_path):

		self.image_list = {}
		if os.path.exists(data_path):
		    for name in os.listdir(data_path):
		        name = six.text_type(name)
		        if self._is_image_file(name):
		        	frame=cv.imread(os.path.join(data_path,name), 1)		
		        	#print(frame.shape)
		        	self.image_list.update({name:frame})

	def save_webcamImage(self, data_path):
		cap=cv.VideoCapture(0)
		i=0
		count=1
		self.image_list={}
		image_path=os.path.join(data_path,'webcam_images')

		if(cap.isOpened()==False):
		    print("Unable to open the webcam!!")

		while(cap.isOpened()):
		    ret, frame=cap.read()
		    if ret==False:
		        break
		   
		    cv.imshow('camera',frame)
		    if i%40==0:
		    	img_name = "{}.jpg".format(count-1)
		    	cv.imwrite(os.path.join(image_path, img_name),frame)
		    	#self.image_list.append(frame)
		    	
		    	self.image_list.update({img_name:frame})
		    	logging.info('Capturing Images for {}'.format(img_name))
		    	
		    	if count%10 ==0:		    		
		    		break
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

