# 2019-11
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

'''
   Quick and dirty script to combine multiple bag files. 
'''

import os
import sys
import shutil
import subprocess
import glob
import time

import rosbag
import rospy


# init
out_bag = False
in_bag_0 = False
in_bag_1 = False
in_bag_2 = False
in_bag_3 = False
in_bag_4 = False
in_bag_5 = False

# process command line
if len(sys.argv) < 2:
    print("Not enough arguments!")
    print("Usage: combine_full_bag_files <options> folder_path")
    exit()

if "-h" in sys.argv:
    print("Usage: combine_full_bag_files <options> folder_path")
    print("Help Description here ******************************* ")
    print(" ")
    print("     -h                     print this help message")
    print("     -o <rosbag_out.bag>    out_bag filename")
    print("     -in0   <in_bag>        filename 0")
    print("     -in1   <in_bag>        filename 1")
    print("     -in2   <in_bag>        filename 2")
    print("     -in3   <in_bag>        filename 3")
    print("     -in4   <in_bag>        filename 4")
    print("     -in5   <in_bag>        filename 5")
    exit()

# path to place outbag
out_bag_path = sys.argv[-1]
if not os.path.isdir(out_bag_path):
    print("Folder '%s' does not exist" % out_bag_path)
    exit()

# outbag filename
if not "-o" in sys.argv:
    out_bag = os.path.join(out_bag_path, "master.bag")

if "-o" in sys.argv:
    i = sys.argv.index("-o")
    if len(sys.argv) > i+1:
        out_bag = os.path.join(out_bag_path, sys.argv[i+1])

# input bag filenames
if "-in0" in sys.argv:
    i = sys.argv.index("-in0")
    if len(sys.argv) > i+1:
      if(os.path.isfile(os.path.join(out_bag_path, sys.argv[i+1]))):
        in_bag_0 = rosbag.Bag(os.path.join(out_bag_path, sys.argv[i+1]))
        
if "-in1" in sys.argv:
    i = sys.argv.index("-in1")
    if len(sys.argv) > i+1:
      if(os.path.isfile(os.path.join(out_bag_path, sys.argv[i+1]))):
        in_bag_1 = rosbag.Bag(os.path.join(out_bag_path, sys.argv[i+1]))

if "-in2" in sys.argv:
    i = sys.argv.index("-in2")
    if len(sys.argv) > i+1:
      if(os.path.isfile(os.path.join(out_bag_path, sys.argv[i+1]))):
        in_bag_2 = rosbag.Bag(os.path.join(out_bag_path, sys.argv[i+1]))

if "-in3" in sys.argv:
    i = sys.argv.index("-in3")
    if len(sys.argv) > i+1:
      if(os.path.isfile(os.path.join(out_bag_path, sys.argv[i+1]))):
        in_bag_3 = rosbag.Bag(os.path.join(out_bag_path, sys.argv[i+1]))

if "-in4" in sys.argv:
    i = sys.argv.index("-in4")
    if len(sys.argv) > i+1:
      if(os.path.isfile(os.path.join(out_bag_path, sys.argv[i+1]))):
        in_bag_4 = rosbag.Bag(os.path.join(out_bag_path, sys.argv[i+1]))

if "-in5" in sys.argv:
    i = sys.argv.index("-in5")
    if len(sys.argv) > i+1:
      if(os.path.isfile(os.path.join(out_bag_path, sys.argv[i+1]))):
        in_bag_5 = rosbag.Bag(os.path.join(out_bag_path, sys.argv[i+1]))

with rosbag.Bag(out_bag, 'w') as outbag:
    # read in bag files and combine data into new bag file
    if in_bag_0:
        for topic, msg, t in in_bag_0.read_messages():
            outbag.write(topic, msg, t)
        print ("combining " + in_bag_0.filename)

    if in_bag_1:
        for topic, msg, t in in_bag_1.read_messages():
            outbag.write(topic, msg, t)
        print ("combining " + in_bag_1.filename)

    if in_bag_2:
        for topic, msg, t in in_bag_2.read_messages():
            outbag.write(topic, msg, t)
        print ("combining " + in_bag_2.filename)

    if in_bag_3:
        for topic, msg, t in in_bag_3.read_messages():
            outbag.write(topic, msg, t)
        print ("combining " + in_bag_3.filename)

    if in_bag_4:
        for topic, msg, t in in_bag_4.read_messages():
            outbag.write(topic, msg, t)
        print ("combining " + in_bag_4.filename)

    if in_bag_5:
        for topic, msg, t in in_bag_5.read_messages():
            outbag.write(topic, msg, t)
        print ("combining " + in_bag_5.filename)


outbag.close()

    
