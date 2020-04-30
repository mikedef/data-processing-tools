# 2020-03
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

'''
   Script to rotate the Velodyne HDL32E such that the x-axis is forward. 2019 and prior Velodyne mounting resulted in y-axis forward.  
'''

import os
import sys
import numpy as np


import tf2_ros
import tf2_py as tf2
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transfrom_cloud
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf
import geometry_msgs.msg
import math
from visual import vector, rotate

stamp = geometry_msgs.msg.TransformStamped()


in_bag = "/home/mikedef/auvlab/asv/2019-10-08/master.bag"
out_bag = "/home/mikedef/auvlab/asv/2019-10-08/rotated_pointcloud.bag"
theta = np.radians(-90)
axis = [4,4,1]


with rosbag.Bag(out_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(in_bag).read_messages():
        if topic == "/velodyne_points":
            gen = pc2.read_points(msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True)

            lidar = np.array(list(gen))
            
            #print(lidar[0])

            list_test = pc2.read_points_list(msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True)
            print (list_test[0])
            #lidar_rot = np.zeros_like(lidar)
            
            #for i, line in enumerate(lidar):
            #    vec = vector(lidar[i])
            #    vec = vector.rotate(vec, theta)
            #    lidar_rot[i] = vec
            
            #print (lidar[0], lidar_rot[0])
            '''
            cloud_msg = PointCloud2()
            header = Header()
            header.stamp = t
            header.frame_id = "velodyne"
            points = pc2.create_cloud_xyz32(header, lidar)
            #points90 = pc2.create_cloud_xyz32(header, lidar_rot)

            outbag.write('/rotate_velo', points, t)
            #outbag.write('/rotate_velo_90', points90, t)
            
            transform = np.identity(3)
            transform[0][0] = np.cos(theta)
            transform[0][1] = np.sin(theta)
            transform[1][0] = np.sin(theta)
            transform[1][1] = np.cos(theta)
            #print ('trans:', transform*lidar[0])
            #print ('rot: ' , rot[0])
            #print ('pc: ', lidar[0])
            '''

outbag.close()
