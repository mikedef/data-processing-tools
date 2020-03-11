# 2020-01
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

'''
   Script to put MOOS variables from a previously processed CSV into a bag file. This will be used to later combine MOOS data with a master mission bag file from Philos BV NMEA messages.
'''

import os
import sys
import math
import csv

import rosbag
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped, PointStamped, Vector3Stamped
import tf

#from rex4_tools.msg import *

# init
degrees2rad = math.pi/180.0
G = 9.8 # m/s
FT_TO_M = 0.3048

out_bag = 'moos_vars.bag'
lat = 0
lon = 0
euler = []
e_x = 0
e_y = 0
e_z = 0
a_x = 0
a_y = 0
a_z = 0
rr = 0
pr = 0
yr = 0
nav_x = 0
nav_y = 0
nav_z = 0
gps_utc = 0
gps_ex = 0
gps_ey = 0
gps_ez = 0
gps_heave = 0
pashr = False
gprmc = False
count = 0
gprmc_count = 0

# process command line
if len(sys.argv) < 2:
    print("Not enough arguments!")
    print("Usage: create_bag_from_moos_var_csv.py <options> folder_path")

if "-h" in sys.argv:
    print("Usage: create_bag_from_moos_var_csv.py <options> out_bag_folder_path")
    print("Script to combine MOOS vars into bag file")
    print(" ")
    print("     -h               print this help message")
    print("     -m               path to moos_vars.csv"  )
    exit()

# Path to put finished rosbag into
out_path = sys.argv[-1]
if not os.path.isdir(out_path):
    print("Folder '%s' does not exist" % out_path)
    exit()

#rosbag_path = "/home/mikedef/auvlab/asv/2019-10-29/LOG_PHILOS__29_10_2019_____16_00_19/"

# path to csv that holds the moos variables
if not "-m" in sys.argv:
    print("Error: No CSV of MOOS variables given")
    exit()

if "-m" in sys.argv:
    i = sys.argv.index("-m")
    if len(sys.argv) > i+1:
        var_csv = sys.argv[i+1]

# out_bag
print("Creating bag from MOOS vars: " + str(os.path.join(out_path, out_bag)))
bag = rosbag.Bag(os.path.join(out_path, out_bag), 'w')

with open(var_csv, 'r') as f:
    reader = csv.reader(f)
    for line in reader:
      try:
        # print(line[0], line[1], line[2])
        unix_time = float(line[0])
        moos_var = str(line[1])
        data = str(line[2])
        data = line[2].rstrip().split(',')
        topic_time = rospy.rostime.Time.from_sec(unix_time)
        
        #if str(moos_var) != "NMEA_RECEIVED":
        #    var_data = float(data[0])
        
        if moos_var == "NAV_X":
            nav_x = float(data[0])
            
        if moos_var == "NAV_Y":
            nav_y = float(data[0])
            
        if moos_var == "NAV_Z":
            nav_z = float(data[0])

        
        # Parse REx GPS pitch roll heading data
        if moos_var == "GPS_NMEA_STRING":
            gps_topic = data[0].rstrip().split('$')
            if gps_topic[1] == "GPRMC":
                #print(data)
                gprmc = True
                
            if gps_topic[1] == "PASHR":
                #print(data)
                
                count += 1
                #print(count)
                
                # gps utc
                gps_utc = data[1]

                # heading/euler z (True Heading)
                gps_ez = float(data[2]) - 90 # rotated gps on REx
                if (gps_ez < 0):
                    gps_ez += 360
                gps_ez = math.fmod(gps_ez, 360.0)
                

                # roll/euler x
                gps_ex = float(data[4])
                #print('roll:', gps_ex)
                # pitch/euler y
                gps_ey = float(data[5])
                #print('pitch:', gps_ey)
                # swap pitch and roll due to rotated gps on REx
                p = gps_ey
                r = gps_ex
                gps_ex = p*math.sin(-1.57) + r*math.cos(-1.57)
                gps_ey = p*math.cos(-1.57) - r*math.sin(-1.57)
                
                
                # heave
                gps_heave = float(data[6])

                pashr = True

                
            
        # TODO: CHeck against old time, might still be getting to many messages per posting
        # Parse NMEA_RECEIVED 
        if moos_var == "NMEA_RECEIVED":
            nmea_topic = data[0]
            # print (data[0])

            # Parse NavSatFix Msg Data
            if nmea_topic == "$BVLAT":
                lat = float(data[5])
                # print (lat, unix_time)

            if nmea_topic == "$BVLON":
                lon = float(data[5])
                # print (lon, unix_time)

            if nmea_topic == "$BVHAE":
                alt = float(data[5]) * FT_TO_M
                # print (alt, unix_time)

            #  if (lat and lon and alt) != 0:
                #  print (lat, lon, alt, unix_time)

            # Parse Philos Imu Msg Data
            if nmea_topic == "$BVROT":
                # roll
                e_x = float(data[3])
                if (e_x > 512.0):
                    e_x -= 1024.0
                # pitch
                e_y = float(data[4])
                if (e_y > 512.0):
                    e_y -= 1024.0
                # heading
                e_z = float(data[5])
                if (e_z > 512.0):
                    e_z -= 1024.0
                e_z = math.fmod(e_z, 360.0)


            if nmea_topic == "$BVACC":
                
                a_x = float(data[3]) * G
                a_y = float(data[4]) * G
                a_z = float(data[5]) * G

            if nmea_topic == "$BVRRX":
                
                rr = float(data[3])
                if rr > 512.0:
                    rr -= 1024.0
                rr = rr * degrees2rad
                
                pr = float(data[4])
                if pr > 512:
                    pr -= 1024
                pr = pr * degrees2rad
                
                yr = float(data[5])
                if yr > 512:
                    yr = 1024
                yr = yr * degrees2rad

            # Parse Twist Msg Data
            if nmea_topic == "$BVVEL":
                # This is the EKF navigation velocity rotated to vessel body axis velocity in the linear X direction: + forward, - aft
                vel_x = float(data[3]) * FT_TO_M
                # Y direction: + starboard, - port
                vel_y = float(data[4]) * FT_TO_M
                # Z direction: + down, - up
                vel_z = float(data[5]) * FT_TO_M


        # Post NavSatFix Msg            
        if (lat and lon and alt) != 0: 
            topic = '/sensor/gps/fix'
            msg = NavSatFix()
            msg.header.stamp = topic_time
            lon_flag = True
            lat_flag = True
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            msg.header.frame_id = 'base_link'
            bag.write(topic, msg, msg.header.stamp)

            lat = 0
            lon = 0
            alt = 0

        # Post Imu Msg                          
        if (e_x and e_y and e_z and a_x and a_y and a_z and rr and pr and yr) !=0:
            # ENU Axis Orientation for ROS Localization Package
            q = tf.transformations.quaternion_from_euler(e_x, -e_y, -e_z)
            topic = '/imu/data/enu'
            imu_msg = Imu()
            # topic_time = #rospy.rostime.Time.from_sec(unix_time)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.linear_acceleration.x = a_x
            imu_msg.linear_acceleration.y = -a_y
            imu_msg.linear_acceleration.z = -a_z
            imu_msg.angular_velocity.x = rr
            # Philos y axis points right, in ROS y axis points left (see REP 103)
            imu_msg.angular_velocity.y = -pr
            # Philos z axis points down, in ROS z axis points up (see REP 103)
            imu_msg.angular_velocity.z = -yr
            imu_msg.header.stamp = topic_time
            imu_msg.header.frame_id = 'base_link'
            bag.write(topic, imu_msg, imu_msg.header.stamp)
            
            # Untransformed data from Philos NED Axis Orientation
            q = tf.transformations.quaternion_from_euler(e_x, e_y, e_z)
            topic = '/imu/data/ned'
            msg = Imu()
            # topic_time = #rospy.rostime.Time.from_sec(unix_time)
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            msg.linear_acceleration.x = a_x
            msg.linear_acceleration.y = a_y
            msg.linear_acceleration.z = a_z
            msg.angular_velocity.x = rr
            msg.angular_velocity.y = pr
            msg.angular_velocity.z = yr
            msg.header.stamp = topic_time
            msg.header.frame_id = 'base_link'
            bag.write(topic, msg, msg.header.stamp)
            
            q = []
            e_x = 0
            e_y = 0
            e_z = 0
            a_x = 0
            a_y = 0
            a_z = 0
            rr = 0
            pr = 0
            yr = 0

        # REx gps rpy to IMU() topic
        if (pashr and gprmc) !=0:
            q = tf.transformations.quaternion_from_euler(gps_ex, gps_ey, gps_ez)
            topic = '/gps/rpy2quaternion'
            msg = Imu()
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            msg.linear_acceleration.x = -1
            msg.linear_acceleration.y = -1
            msg.linear_acceleration.z = -1
            msg.angular_velocity.x = -1
            msg.angular_velocity.y = -1
            msg.angular_velocity.z = -1
            msg.header.stamp = topic_time
            msg.header.frame_id = 'base_link'
            bag.write(topic, msg, msg.header.stamp)

            # Vector3 msg to hold roll, pitch, heave data
            topic = '/gps/attitude'
            msg = Vector3Stamped()
            msg.vector.x = gps_ex
            msg.vector.y = gps_ey
            msg.vector.z = gps_heave
            msg.header.stamp = topic_time
            msg.header.frame_id = 'base_link'
            bag.write(topic, msg, msg.header.stamp)

            # Vector3 msg to hold rpy data
            topic = '/gps/rpy'
            msg = Vector3Stamped()
            msg.vector.x = gps_ex
            msg.vector.y = gps_ey
            msg.vector.z = gps_ez
            msg.header.stamp = topic_time
            msg.header.frame_id = 'base_link'
            bag.write(topic, msg, msg.header.stamp)
            
            q = []
            gps_ex = 0
            gps_ey = 0
            gps_ez = 0
            gps_heave = 0
            pashr = False
            gprmc = False
            gprmc_count += 1
            #print('gprmc', gprmc_count)
            
        # Post Twist Msg
        if (vel_x and vel_y and vel_z) !=0:

            topic = '/linear_velocity'
            msg = TwistStamped()
            msg.twist.linear.x = vel_x
            msg.twist.linear.y = vel_y
            msg.twist.linear.z = vel_z
            msg.header.stamp = topic_time
            msg.header.frame_id = 'base_link'
            bag.write(topic, msg, msg.header.stamp)
            
            vel_x, vel_y, vel_z = 0
            
                
      except:
        continue  # must not have been in format time, string, float or moos_var not in dictionary
            
# ********** Don't forget to close ****************
f.close()
bag.close()

exit()


        
    
        



