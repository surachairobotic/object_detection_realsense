#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import cv2

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

sys.path.append("..")
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageHandle:
    def __init__(self):
        self.bridge = CvBridge()

    def process(self, color, depth, info):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(color, "bgr8")
        except CvBridgeError as e:
          print(e)

        
    
tp_caminfo = '/camera/aligned_depth_to_color/camera_info'
tp_depth = '/camera/aligned_depth_to_color/image_raw'
tp_color = '/camera/color/image_raw'

info = None
depth = None
color = None

def main():
    global color, depth, info

    bag = rosbag.Bag('/home/probook/catkin_ws/src/datmo/data/overpass_m70-76.bag')

    myImg = ImageHandle()

    cnt=0
    for topic, msg, t in bag.read_messages(topics=[tp_caminfo, tp_depth, tp_color]):
        if topic.find(tp_caminfo) != -1:
            info = msg
        elif topic.find(tp_depth) != -1:
            depth = msg
        elif topic.find(tp_color) != -1:
            color = msg
        else:
            print('%s, %s' % (t, topic))
        
        if info and depth and color:
            cnt=cnt+1
            print(cnt)
            #myImg.process(color, depth, info)
            info = depth = color = None

    bag.close()

if __name__ == "__main__":
    main()
