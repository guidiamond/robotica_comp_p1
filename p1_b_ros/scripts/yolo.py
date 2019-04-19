#! /usr/bin/env python
# -*- coding:utf-8 -*-

"""

Adapted from:

https://github.com/iArunava/YOLOv3-Object-Detection-with-OpenCV/blob/master/yolo.py#L154

"""


import numpy as np
import argparse
import cv2 as cv
import subprocess
import time
import os
from yolo_utils import infer_image, show_image
import imutils

import rospkg
import os

rospack = rospkg.RosPack()
path = rospack.get_path('p1_b_ros')
scripts = os.path.join(path,  "scripts")

class Object(object):
    pass

FLAGS = Object()
FLAGS.confidence = 0.5
FLAGS.threshold = 0.3
FLAGS.show_time = True

labels = None
colors = None
net = None 
vid = None
count = None 
layer_names = None
count = 0


def print_categories(boxes, confidences, classids, labels):
	""" 
	Nota do Miranda: 
		Função estratégica para modificar/ usar o Yolo
	"""
	for i in range(len(classids)):
		print(labels[classids[i]], confidences[i], boxes[i])



model_path = os.path.join(scripts, "yolov3-coco/")

weights = os.path.join(scripts, "yolov3-coco/yolov3.weights")

config = os.path.join(scripts,"yolov3-coco/yolov3.cfg")

labels_file = os.path.join(scripts, "yolov3-coco/coco-labels")


def pre_yolo():


	global labels 
	global colors
	global net
	global vid
	global count
	global layer_names 


	# Get the labels
	labels = open(labels_file).read().strip().split('\n')

	# Known categories
	print("Known categories: ", " ".join(labels))

	# Intializing colors to represent each label uniquely
	colors = np.random.randint(0, 255, size=(len(labels), 3), dtype='uint8')

	# Load the weights and configutation to form the pretrained YOLOv3 model
	net = cv.dnn.readNetFromDarknet(config, weights)

	# Get the output layer names of the model
	layer_names = net.getLayerNames()
	layer_names = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        


def classify(frame_bgr):

	global count

	frame = imutils.resize(frame_bgr, width=600)


	height, width = frame.shape[:2]

	boxes, confidences, classids = [],[],[]

	results = []


	frame_out, boxes, confidences, classids, idxs = infer_image(net, layer_names, \
						height, width, frame, colors, labels, FLAGS)


	height, width = frame.shape[:2]

	if count == 0:
		frame, boxes, confidences, classids, idxs = infer_image(net, layer_names, \
							height, width, frame, colors, labels, FLAGS)
		count += 1
	else:
		frame, boxes, confidences, classids, idxs = infer_image(net, layer_names, \
							height, width, frame, colors, labels, FLAGS, boxes, confidences, classids, idxs, infer=False)
		count = (count + 1) % 6

	print_categories(boxes, confidences, classids, labels)

	results = []


	for i in range(len(classids)):
		results.append((labels[classids[i]], confidences[i], boxes[i]))

	frame_out = frame.copy()

	return frame_out, results


