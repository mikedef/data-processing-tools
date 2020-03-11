# 2019-10
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

''' 
   Script to extract image topics, 2019 Philos or REx, and rotate the images due to how the sensors were mounted. The extracted images will be saved to a directory of the topic name. Lastly a video will be created from the extracted images and will also be saved to the directory. '''


import os
import sys
import shutil
import subprocess

import rosbag
import rospy
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

# init
general_topic = False
left_camera_topic = False
center_camera_topic = False
right_camera_topic = False
ir_camera_topic = False

# Helpers
def create_folder(source, folder, topic):
    os.chdir(rosbag_folder_path)  # change into folder where rosbag is located
    if os.path.isdir(folder): # if folder is already there remove
        shutil.rmtree(folder)
    try:
        os.mkdir(folder) # if folder is not there make
    except:
        pass

# process command line
if len(sys.argv) < 2:
    print ("Not enough arguments!")
    print ("Usage: extract_images <options> rosbag_folder_path")
    exit()

if "-h" in sys.argv:
    print ("Usage: extract_images <options> rosbag_folder_path")
    print ("Script processes given topic from rosbag file into images and video into topic directories")
    print ("*** Note *** This is tailored around cameras from the 2019 REx/Philos ASVs")
    print ("     -h          print this help message")
    print ("     -b <file>   rosbag file to call")
    print ("     -t <topic>  rostopic to extract")
    print ("     -lc         /left_camera topic")
    print ("     -cc         /center_camera topic")
    print ("     -rc         /right_camera topic")
    print ("     -ir         /flir_boson topic")
    exit()

# rosbag source folder must be last option
rosbag_folder_path = sys.argv[-1]
if not os.path.isdir(rosbag_folder_path):
    print ("Folder '%s' does not exist" % rosbag_folder_path)
    exit()
    
# rosbag
if not "-b" in sys.argv:
    print ("Error: No bag file given")
    exit()
    
if "-b" in sys.argv:
    i = sys.argv.index("-b")
    if len(sys.argv) > i+1:
        bag_file = sys.argv[i+1] # TODO: should probably check if this is valid *******

# general topic
if "-t" in sys.argv:
    general_topic = True
    i = sys.argv.index("-t")
    if len(sys.argv) > i+1:
        image_topic = sys.argv[i+1]

# left camera
if "-lc" in sys.argv:
    left_camera_topic = True
    left_camera = "/left_camera/image_color/compressed"
    left_camera_folder = "left_camera"
    left_camera_path = os.path.join(rosbag_folder_path, left_camera_folder)
    create_folder(rosbag_folder_path, left_camera_folder, left_camera)

# center camera
if "-cc" in sys.argv:
    center_camera_topic = True
    center_camera = "/center_camera/image_color/compressed"
    center_camera_folder = "center_camera"
    center_camera_path = os.path.join(rosbag_folder_path, center_camera_folder)
    create_folder(rosbag_folder_path, center_camera_folder, center_camera)    

# right camera
if "-rc" in sys.argv:
    right_camera_topic = True
    right_camera = "/right_camera/image_color/compressed"
    right_camera_folder = "right_camera"
    right_camera_path = os.path.join(rosbag_folder_path, right_camera_folder)
    create_folder(rosbag_folder_path, right_camera_folder, right_camera)

# ir camera
if "-ir" in sys.argv:
    ir_camera_topic = True
    ir_camera = "/flir_boson/image_raw"
    ir_camera_folder = "ir_camera"
    ir_camera_path = os.path.join(rosbag_folder_path, ir_camera_folder)
    create_folder(rosbag_folder_path, ir_camera_folder, ir_camera)    
    
if general_topic:
    # create general image topic folder 
    strip = image_topic.strip('/')
    topic_folder = strip.split('/')[0]
    os.chdir(rosbag_folder_path)  # change into folder where rosbag is located
    if os.path.isdir(topic_folder): # if folder is already there remove
        shutil.rmtree(topic_folder)
    try:
        os.mkdir(topic_folder) # if folder is not there make
    except:
        pass


    video_path = rosbag_folder_path + topic_folder + '/'

# process rosbag topic into images and video
bag_path = os.path.join(rosbag_folder_path, bag_file)
bag = rosbag.Bag(bag_path)

# bridge between openCV and rostopic image message
br = CvBridge()

print("extracting images from " + bag.filename)
for topic, msg, t in bag.read_messages():

    if left_camera_topic:
        # Process Left Camera Data
        if topic == left_camera:
            #print ("topic: ", topic, "time: ", t.secs)

            cv_compressed_image = br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Get image height and width
            (h, w) = cv_compressed_image.shape[:2]
            # Calculate the center of the image
            center = (w/2 , h/2)
            # Angle to rotate in degrees
            angle = 180
            # Scale
            scale = 1.0
            # Perform counter clockwise rotation holding at the center
            M = cv.getRotationMatrix2D(center, angle, scale)
            rotated_img = cv.warpAffine(cv_compressed_image, M, (w, h))

            # Get timestamp from image message header and set image name
            timestr = "%.6f.jpg" % msg.header.stamp.to_sec()
            image_name = rosbag_folder_path + left_camera_folder + '/' + timestr
            os.chdir(left_camera_path) # change into topic folder
            cv.imwrite(timestr, rotated_img)

    if center_camera_topic:
        # Process Center Camera Data
        if topic == center_camera:
            #print ("topic: ", topic, "time: ", t.secs)

            cv_compressed_image = br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Get image hight and width
            (h, w) = cv_compressed_image.shape[:2]
            # Calculate the center of the image
            center = (w/2 , h/2)
            # Angle to rotate in degrees
            angle = 180
            # Scale
            scale = 1.0
            # Perform counter clockwise rotation holding at the center
            M = cv.getRotationMatrix2D(center, angle, scale)
            rotated_img = cv.warpAffine(cv_compressed_image, M, (w, h))

            # Get timestamp from image message header and set image name
            timestr = "%.6f.jpg" % msg.header.stamp.to_sec()
            image_name = rosbag_folder_path + center_camera_folder + '/' + timestr
            os.chdir(center_camera_path) # change into topic folder
            cv.imwrite(timestr, rotated_img)

    if right_camera_topic:
        # Process Right Camera Data
        if topic == right_camera:
            #print ("topic: ", topic, "time: ", t.secs)

            cv_compressed_image = br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Get image hight and width
            (h, w) = cv_compressed_image.shape[:2]
            # Calculate the center of the image
            center = (w/2 , h/2)
            # Angle to rotate in degrees
            angle = 180
            # Scale
            scale = 1.0
            # Perform counter clockwise rotation holding at the center
            M = cv.getRotationMatrix2D(center, angle, scale)
            rotated_img = cv.warpAffine(cv_compressed_image, M, (w, h))

            # Get timestamp from image message header and set image name
            timestr = "%.6f.jpg" % msg.header.stamp.to_sec()
            image_name = rosbag_folder_path + right_camera_folder + '/' + timestr
            os.chdir(right_camera_path) # change into topic folder
            cv.imwrite(timestr, rotated_img)

    if ir_camera_topic:
        # Process Right Camera Data
        if topic == ir_camera:
            #print ("topic: ", topic, "time: ", t.secs)

            cv_image = br.imgmsg_to_cv2(msg)
            # Get image hight and width
            (h, w) = cv_image.shape[:2]
            # Calculate the center of the image
            center = (w/2 , h/2)
            # Angle to rotate in degrees
            angle = 180
            # Scale
            scale = 1.0
            # Perform counter clockwise rotation holding at the center
            M = cv.getRotationMatrix2D(center, angle, scale)
            rotated_img = cv.warpAffine(cv_image, M, (w, h))

            # Get timestamp from image message header and set image name
            timestr = "%.6f.jpg" % msg.header.stamp.to_sec()
            image_name = rosbag_folder_path + ir_camera_folder + '/' + timestr
            os.chdir(ir_camera_path) # change into topic folder
            cv.imwrite(timestr, rotated_img)
            
    # Process general image topic
    if general_topic:
        if topic == image_topic:
            #print ('topic: ', topic, 'Time; ', t.secs)
        
            #cv_image = br.imgmsg_to_cv2(msg)
            cv_compressed_image = br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Get image hight and width
            (h, w) = cv_compressed_image.shape[:2]
            # Calculate the center of the image
            center = (w/2 , h/2)
            # Angle to rotate in degrees
            angle = 180
            # Scale
            scale = 1.0
            # Perform counter clockwise rotation holding at the center
            M = cv.getRotationMatrix2D(center, angle, scale)
            rotated_img = cv.warpAffine(cv_compressed_image, M, (w, h))

            # Get timestamp from image message header and set image name
            timestr = "%.6f.jpg" % msg.header.stamp.to_sec()
            image_name = rosbag_folder_path + topic_folder + '/' + timestr
            cv.imwrite(image_name, rotated_img)



if left_camera_topic:
    print ("processing left camera video") 
    os.chdir(left_camera_path)
    if len(os.listdir(left_camera_path)) != 0:
      subprocess.call(["ffmpeg", "-loglevel", "16", "-r", "12", "-f", "image2", "-s", "1024x1280", "-pattern_type", "glob", "-i", "*.jpg", "-vcodec", "libx264", "-crf", "25", "-pix_fmt", "yuv420p", left_camera_folder + "_VID.mp4"])

if center_camera_topic:
    print ("processing center camera video") 
    os.chdir(center_camera_path)
    if len(os.listdir(center_camera_path)) != 0:
      subprocess.call(["ffmpeg", "-loglevel", "16", "-r", "12", "-f", "image2", "-s", "1024x1280", "-pattern_type", "glob", "-i", "*.jpg", "-vcodec", "libx264", "-crf", "25", "-pix_fmt", "yuv420p", center_camera_folder + "_VID.mp4"])

if right_camera_topic:
    print ("processing right camera video") 
    os.chdir(right_camera_path)
    if len(os.listdir(right_camera_path)) != 0:
      subprocess.call(["ffmpeg", "-loglevel", "16", "-r", "12", "-f", "image2", "-s", "1024x1280", "-pattern_type", "glob", "-i", "*.jpg", "-vcodec", "libx264", "-crf", "25", "-pix_fmt", "yuv420p", right_camera_folder + "_VID.mp4"])

if ir_camera_topic:
    print ("processing ir camera video") 
    os.chdir(ir_camera_path)
    if len(os.listdir(ir_camera_path)) != 0:
      subprocess.call(["ffmpeg", "-loglevel", "16", "-r", "30", "-f", "image2", "-s", "512x640", "-pattern_type", "glob", "-i", "*.jpg", "-vcodec", "libx264", "-crf", "25", "-pix_fmt", "yuv420p", ir_camera_folder + "_VID.mp4"])

exit()



# Create video from images
print("processing video")
os.chdir(video_path)  
subprocess.call(["ffmpeg", "-loglevel", "16", "-r", "12", "-f", "image2", "-s", "1024x1024", "-pattern_type", "glob", "-i", "*.jpg", "-vcodec", "libx264", "-crf", "25", "-pix_fmt", "yuv420p", "-r", "12", topic_folder + "_VID.mp4"])

# Write new bagfile while extracting all params from original bagfile
#with rosbag.Bag('test.bag', 'w') as outbag:
#    for topic, msg, t in bag.read_messages():
#        outbag.write(topic, msg, t)
    
