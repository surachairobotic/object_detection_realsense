#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import rospy
import time

import rosbag
from sensor_msgs.msg import Image, CameraInfo

tp_caminfo = '/camera/aligned_depth_to_color/camera_info'
tp_depth = '/camera/aligned_depth_to_color/image_raw'
tp_color = '/camera/color/image_raw'

def main():
    global color, depth, info

    pub_info = rospy.Publisher("/datmo/camera_info", CameraInfo, queue_size=10);
    pub_color = rospy.Publisher("/datmo/color", Image, queue_size=10);

    bag = rosbag.Bag('/home/probook/catkin_ws/src/datmo/data/mtec_m55.bag')

    for topic, msg, t in bag.read_messages(topics=[tp_caminfo, tp_depth, tp_color]):
        if topic.find(tp_caminfo) != -1:
            info = msg
            pub_info.publish(info)
            rospy.sleep(0.1)
        elif topic.find(tp_depth) != -1:
            depth = msg
        elif topic.find(tp_color) != -1:
            color = msg
            pub_color.publish(color)
            rospy.sleep(0.1)
        else:
            print('%s, %s' % (t, topic))
        

    bag.close()


if __name__ == "__main__":
    rospy.init_node('read_bag', anonymous=True)
    main()
