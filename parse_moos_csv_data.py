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

# init
degrees2rad = math.pi/180.0
rad2degrees = 180.0/math.pi
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
    print("Usage: parse_moos_csv_data.py <options> folder_path")

if "-h" in sys.argv:
    print("Usage: parse_moos_csv_data.py <options> out_bag_folder_path")
    print("Script to parse moos vars into a csv")
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
        
imu = {"time":0.0,"roll":0.0, "pitch":0.0, "yaw":0.0, "roll_mod":0.0, "pitch_mod":0.0, "yaw_mod":0.0}
with open('output.csv', 'w') as csv_file:
    #writer = csv.writer(csv_file)
    csv_writer = csv.DictWriter(csv_file, fieldnames=["time","roll","pitch","yaw","roll_mod","pitch_mod","yaw_mod"])
    csv_writer.writeheader()
    
    with open(var_csv,'r') as f:
        reader = csv.reader(f)
        for line in reader:
            try:
                #print(line[0], line[1], line[2])
                
                unix_time = float(line[0])
                moos_var = str(line[1])
                data = str(line[2])
                data = line[2].rstrip().split(',')
                
                imu["time"] = unix_time

                
                if moos_var == "NMEA_RECEIVED":
                    nmea_topic = data[0]
                
                # Parse Philos Imu Msg Data
                if nmea_topic == "$BVROT":
                    # roll
                    e_x = float(data[3])
                    if (e_x > 512.0):
                        e_x -= 1024.0
                    imu["roll"] = e_x
                    
                    # pitch
                    e_y = float(data[4])
                    if (e_y > 512.0):
                        e_y -= 1024.0
                    imu["pitch"] = e_y
                    
                    # heading
                    e_z = float(data[5])
                    if (e_z > 512.0):
                        e_z -= 1024.0
                        e_z = math.fmod(e_z, 360.0)
                    imu["yaw"] = e_z

                    radx = e_x * degrees2rad
                    rady = e_y * degrees2rad
                    radz = e_z * degrees2rad
                    q = tf.transformations.quaternion_from_euler(radx,rady,radz)
                    euler = tf.transformations.euler_from_quaternion([q[0],q[1],q[2],q[3]])[:3]
                    imu["roll_mod"] = euler[0]*rad2degrees
                    imu["pitch_mod"] = euler[1]*rad2degrees
                    imu["yaw_mod"] = euler[2]*rad2degrees
                    x = euler[0]*rad2degrees
                    y = euler[1]*rad2degrees
                    z = euler[2]*rad2degrees
                    print (imu)
                    print (x,y,z)
                    csv_writer.writerow(imu)

            except:
                continue  # must not have been in format time, string, float or moos_var not in dictionary
            


exit()


        
    
        



