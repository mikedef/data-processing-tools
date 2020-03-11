# 2020-02
# Author: Michael Sacarny (msacarny@mit.edu), AUV Lab
# License: MIT

''' 
   Script to extract individual parameters from rosbag and convert to csv file in given folder '''

import os
import sys
import math
import csv
import array

import rosbag
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped, PointStamped
from rex4_tools.msg import RadarSegment
from rex4_tools.msg import RadarSpoke 
import tf

# globals
imu_f = ""
lv_f = ""
gps_f = ""
rad_f0 = ""
rad_f1 = ""
t_list = []


# process command line
if len(sys.argv) < 4:
    print ("Not enough arguments!")
    print ("Usage: rosbag_to_csv <options> output_folder")
    exit()

if "-h" in sys.argv:
    print ("Usage: rosbag_to_csv <options> output_folder")
    print ("Script extracts given topics from rosbag file into csv files")
    print ("     -h          print this help message")
    print ("     -b <file>   rosbag file to call")
    print ("     -imu        extract /imu/data/imu topic")
    print ("     -lv         extract /linear_velocity topic")
    print ("     -gps        extract /sensor/gps/fix topic")
    print ("     -rad        extract /broadband_radar/channel_0 & channel_1/segment topics")
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
    
i = sys.argv.index("-b")
if len(sys.argv) > i+1:
  bag_file = sys.argv[i+1] 
  if not os.path.isfile(bag_file):
    print ("Error: missing bagfile: ", bag_file)


imu  = ['-imu', '/imu/data/ned',                      os.path.join(rosbag_folder_path, 'imu.csv'),              imu_f ]
lv   = ['-lv',  '/linear_velocity',                   os.path.join(rosbag_folder_path, 'linear_velocity.csv'),  lv_f  ]
gps  = ['-gps', '/sensor/gps/fix',                    os.path.join(rosbag_folder_path, 'gps.csv'),              gps_f ]
rad0 = ['-rad', '/broadband_radar/channel_0/segment', os.path.join(rosbag_folder_path, "radar_0_spokes.csv"), rad_f0]
rad1 = ['-rad', '/broadband_radar/channel_1/segment', os.path.join(rosbag_folder_path, "radar_1_spokes.csv"), rad_f1]

topics = [imu, lv, gps, rad0, rad1]

# set topic names and file handles
for topic in topics:
  if topic[0] in sys.argv:
    t_list.append(topic[1])
    if os.path.isfile(topic[2]):
      os.remove(topic[2])
    topic[3] = open(topic[2],"w+")

# process rosbag topic into images and video
bag = rosbag.Bag(bag_file)

print("extracting data from " + bag.filename)
for topic, msg, t in bag.read_messages(topics=t_list):
  
  if topic == imu[1]:
      line = '{:.9f}, {}, '.format(msg.header.stamp.to_sec(), msg.header.frame_id)
      line += '{:.13f}, {:.13f}, {:.13f}, {:.13f}, '.format(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
      line += '{:.9f}, {:.9f}, {:.9f}, '.format(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
      line += '{:.15f}, {:.15f}, {:.15f}\n'.format(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
      imu[3].write(line)
                          
  if topic == lv[1]:
      line = '{:.9f}, {}, '.format(msg.header.stamp.to_sec(), msg.header.frame_id)
      line += '{:.8f}, {:.8f}, {:.8f}\n'.format(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
      lv[3].write(line)
         
  if topic == gps[1]:
      line = '{:.9f}, {}, '.format(msg.header.stamp.to_sec(), msg.header.frame_id)          
      line += '{:.8f}, {:.8f}, {:.11f}\n'.format(msg.latitude, msg.longitude, msg.altitude)
      gps[3].write(line)
            
  if topic == rad0[1]:
    for spoke in msg.spokes:
        line = '{:.9f}, {}, '.format(msg.header.stamp.to_sec(), msg.header.frame_id)
        line += '{:.2f}, {:.0f}'.format(spoke.angle, spoke.max_range)     
        arr = array.array("B", spoke.data)
        for a in arr: 
          line += ', {:d}'.format(a)
        line += '\n'
        rad0[3].write(line)       
         
  if topic == rad1[1]:
    for spoke in msg.spokes:
        line = '{:.9f}, {}, '.format(msg.header.stamp.to_sec(), msg.header.frame_id)
        line += '{:.2f}, {:.0f}'.format(spoke.angle, spoke.max_range)     
        arr = array.array("B", spoke.data)
        for a in arr: 
          line += ', {:d}'.format(a)
        line += '\n'
        rad1[3].write(line)       
       
bag.close()        
rad1[3].close()
rad0[3].close()
gps[3].close()
lv[3].close()
imu[3].close()

exit()



