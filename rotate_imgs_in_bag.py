# 2019-10
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

'''
   Script to rotate images from cameras on the 2019 Philos and REx vessels. Will produce a new bagfile with all the original data with images rotated on the original topic name. 
'''

import os, sys, time
import rospy
import roslib
import rosbag
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


#init
br = CvBridge()
left_camera = False
right_camera = False
center_camera = False
flir_camera = False

# process command line
if len(sys.argv) < 2:
    print("Not enough arguments!")
    print("Usage: rotate_imgs_in_bag.py <options> folder_path")
    exit()

if "-h" in sys.argv:
    print("Usage: rotate_imgs_in_bag.py <options> folder_path")
    print("Provide flags for each camera in data set depending on which vessel data was taken from")
    print(" ")
    print("     -h               print out this help message")
    print("     -f               subscribe to flir camera"   )
    print("     -l               subscribe to left camera"   )
    print("     -r               subscribe to right camera"  )
    print("     -c               subscribe to center camera" )
    print("     -b               bag to rotate images"       )

# path to place outbag
out_bag_path = sys.argv[-1]
if not os.path.isdir(out_bag_path):
    print("Folder '%s' does not exist" % out_bag_path)
    exit()

out_bag = os.path.join(out_bag_path, "full_bag_with_rotated_images.bag")

if not "-b" in sys.argv:
    print("Error: No in_bag filename given")
    exit()

  
if "-b" in sys.argv:
    i = sys.argv.index("-b")
    if len(sys.argv) > i+1:
        in_bag = rosbag.Bag(sys.argv[i+1])

if "-f" in sys.argv:
    i = sys.argv.index("-f")
    flir_camera = "/flir_boson/image_raw"

if "-l" in sys.argv:
    i = sys.argv.index("-l")
    left_camera = "/left_camera/image_color/compressed"

if "-r" in sys.argv:
    i = sys.argv.index("-r")
    right_camera = "/right_camera/image_color/compressed"

if "-c" in sys.argv:
    i = sys.argv.index("-c")
    center_camera = "/center_camera/image_color/compressed"

def rotate_compressed_image(msg):
    
    # Convert to CV2
    cv2_img = br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # Get image height and width
    (h, w) = cv2_img.shape[:2]
    # Calculate the center of the image
    center = (w/2 , h/2)
    # Angle to rotate in degrees
    angle = 180
    # Scale
    scale = 1.0
    # Perform counter clockwise rotation holding at the center
    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated_img = cv2.warpAffine(cv2_img, M, (w, h))
    
    # cv to ros data type and publish
    ros_msg = CompressedImage()
    ros_msg.header.stamp = msg.header.stamp
    ros_msg.format = "jpeg"
    ros_msg.data = np.array(cv2.imencode('.jpg', rotated_img)[1]).tostring()

    return ros_msg            
        
def rotate_flir_image(msg):
            
    # Convert to CV2
    cv2_img = br.imgmsg_to_cv2(msg, 'mono8')
    # Get image height and width
    (h, w) = cv2_img.shape[:2]
    # Calculate the center of the image
    center = (w/2 , h/2)
    # Angle to rotate in degrees
    angle = 180
    # Scale
    scale = 1.0
    # Perform counter clockwise rotation holding at the center
    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated_img = cv2.warpAffine(cv2_img, M, (w, h))
    
    # cv to ros data type and publish
    ros_msg = br.cv2_to_imgmsg(rotated_img, 'mono8')
    ros_msg.header.stamp = msg.header.stamp
    
    return ros_msg

print ('Rotating images into ' + in_bag.filename)    

with rosbag.Bag(out_bag, 'w') as outbag:
    for topic, msg, t in in_bag.read_messages():
        if topic == left_camera or topic == right_camera or topic == center_camera:
            #print(topic)
            img_msg = rotate_compressed_image(msg)
            outbag.write(topic, img_msg, t)
            continue

        if topic == flir_camera:
            #print(topic)
            img_msg = rotate_flir_image(msg)
            outbag.write(topic, img_msg, t)
            continue
            
        else:
            outbag.write(topic, msg, t)

outbag.close()



