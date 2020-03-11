#!/usr/bin/env python
# 2019-10
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

''' 
   Script to create a bag file from timestamped images. Specifically the output of our radar processing script from MOOS, but could be applied to other timestamped images'''

import os
import sys
import shutil
import subprocess
import glob
import time

import rosbag
import rospy
#import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# init
radar_chan0 = False
radar_chan1 = False

# process command line
if len(sys.argv) < 2:
    print ("Not enough arguments!")
    print ("Usage: create_bag_from_images <options> folder_path")
    exit()

if "-h" in sys.argv:
    print ("Usage: create_bag_from_images <options> folder_path")
    print ("Last argument is the path to the radar images parent directory, assumes rosbag file is in this directory with RADRAW0 and/or RADRAW1 as child directories")
    print ("")
    print ("     -h          print this help message")
    print ("     -c0         radar channel 0 images")
    print ("     -c1         radar channel 1 images")
    print ("     -b <file>   rosbag file to extract") 
    
    exit()

# rosbag source folder must be last option
rosbag_path = sys.argv[-1]
if not os.path.isdir(rosbag_path):
    print ("Folder '%s' does not exist" % rosbag_path)
    exit()

# rosbag
#if not "-b" in sys.argv:
#    print ("Error: No bag file given")
#    exit()

if "-b" in sys.argv:
    i = sys.argv.index("-b")
    if len(sys.argv) > i+1:
        in_bag_file = sys.argv[i+1]

if "-c0" in sys.argv:
    radar_chan0 = True

if "-c1" in sys.argv:
    radar_chan1 = True

br = CvBridge()

# Write new bagfile while extracting all params from original bagfile
with rosbag.Bag(os.path.join(rosbag_path, 'radar_images.bag'), 'w') as outbag:

  print ("Creating bag file from radar images: " + outbag.filename)
  if radar_chan0:     
    # for each image, convert time and stuff into bag
    for file in glob.glob(os.path.join(rosbag_path, "RADRAW0/*.png")):
      img = cv2.imread(file)
      name = file.strip(os.path.join(rosbag_path, "RADRAW0"))
      name = name.strip('radar_')
      name = name.strip(".png")
      img_time = rospy.rostime.Time.from_sec(float(name))
      img_msg = br.cv2_to_imgmsg(img, "bgr8")
      img_msg.header.stamp = img_time
      img_msg.header.frame_id = 'radar/0'
      outbag.write('/broadband_radar/channel_0/image_raw', img_msg, img_msg.header.stamp)
    
  if radar_chan1:     
    # for each image, convert time and stuff into bag
    for file in glob.glob(os.path.join(rosbag_path, "RADRAW1/*.png")):
      img = cv2.imread(file)
      name = file.strip(os.path.join(rosbag_path, "RADRAW1"))
      name = name.strip('radar_')
      name = name.strip(".png")
      img_time = rospy.rostime.Time.from_sec(float(name))
      img_msg = br.cv2_to_imgmsg(img, "bgr8")
      img_msg.header.stamp = img_time
      img_msg.header.frame_id = 'radar/1'
      outbag.write('/broadband_radar/channel_1/image_raw', img_msg, img_msg.header.stamp)
            
  outbag.close()      
      
