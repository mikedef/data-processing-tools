# 2020-02
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

'''
    Script to take radar segment data (bundles of spokes) to create a bag file 
'''

import os
import sys
import csv

import rosbag
import rospy
from rex4_tools.msg import RadarSpoke
from rex4_tools.msg import RadarSegment

#init

out_bag = 'radar_segments.bag'
r_csv = []

# process command line
if len(sys.argv) < 2:
    print("Not enough arguments!")
    print("Usage: create_bag_from_moos_var_csv.py <options> folder_path")

if "-h" in sys.argv:
    print("Usage: create_bag_from_moos_var_csv.py <options> out_bag_folder_path")
    print("Script to combine MOOS vars into bag file")
    print(" ")
    print("     -h         print this help message")
    print("     -r0        process channel 0"  )
    print("     -r1        process channel 1"  )
    exit()

# Path to put finished rosbag into
out_path = sys.argv[-1]
if not os.path.isdir(out_path):
    print("Folder '%s' does not exist" % out_path)
    exit() 

# radar0.csv
if not "-r0" in sys.argv:
    print("Error: No CSV of Radar0 given")
    exit()

if "-r0" in sys.argv:
    #i = sys.argv.index("-r0")
    #if len(sys.argv) > i+1:
    #    r_csv.append(sys.argv[i+1])
   csv_file = os.path.join(out_path, "RADRAW0.csv")
   if os.path.isfile(csv_file):
     r_csv.append(csv_file)
   else:
     print("Error: Radar CSV file does not exist: %s" % csv_file) 

# radar1.csv
if not "-r1" in sys.argv:
    print("Warning: No CSV of Radar1 given")
    exit()

if "-r1" in sys.argv:
    #i = sys.argv.index("-r1")
    #if len(sys.argv) > i+1:
    #    r_csv.append(sys.argv[i+1])
    csv_file = os.path.join(out_path, "RADRAW1.csv")
    if os.path.isfile(csv_file):
      r_csv.append(csv_file)
    else:
      print("Warning: Radar CSV file does not exist: %s" % csv_file)

if (len(r_csv) == 0):
    print("Error: No radar segments to process to bag file")
    exit()

# out_bag
print("Creating bag from radar spokes: " + str(os.path.join(out_path, out_bag)))
bag = rosbag.Bag(os.path.join(out_path, out_bag), 'w')

for i, csv_path in enumerate(r_csv):
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        
        old_time = ""
        segment = RadarSegment()
        
        for line in reader:
            '''           
            unix_time = float(line[0])
            angle = float(line[1])
            max_range = float(line[2])
            points = []
            for point in line[3:]: # skip first 3 data points
                points.append(int(point))

            topic = '/broadband_radar/channel_' + str(i) + '/segment'
            msg = RadarSpoke()
            msg.header.stamp = rospy.rostime.Time.from_sec(unix_time)
            msg.angle = angle
            msg.max_range = max_range
            msg.data = points
            msg.header.frame_id = '/base_link'
            bag.write(topic, msg, msg.header.stamp)

            '''
          
            unix_time = float(line[0])
            if (old_time != "") and (unix_time != old_time):
              # publish segment
              topic = '/broadband_radar/channel_' + str(i) + '/segment'
              segment.header.stamp = rospy.rostime.Time.from_sec(unix_time)
              segment.header.frame_id = str(i)
              bag.write(topic, segment, segment.header.stamp)
              segment = RadarSegment()
            
            # build up segment
            spoke = RadarSpoke()
            spoke.angle = float(line[1])
            spoke.max_range = float(line[2])
            points = []
            for point in line[3:]: # skip first 3 data points
                points.append(int(point))
            spoke.data = points
            segment.spokes.append(spoke)
            old_time = unix_time
f.close()
bag.close()
