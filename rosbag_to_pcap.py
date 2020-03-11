#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import rosbag
from velodyne_msgs.msg import VelodyneScan
import signal
import binascii
import os

'''
Modified to run off a rosbag an not as a standalone ros node

Script to convert ROS bag Velodyne data to .pcap format. Usage: -h

'''

#Global header for pcap 2.4
#pcap format: https://wiki.wireshark.org/Development/LibpcapFileFormat
#pcap global heaeder = 0x + d4c3b2a1 (magic_number) + 0200 (version_major) + 0400 (version_minor) + 00000000 (thiszone) + 00000000 (sigfigs) + ffff0000 (snaplen) + 01000000 (network)
global_header =  'd4c3b2a1020004000000000000000000ffff000001000000' # 24 bytes = 48 characters (once at file start)
#pcap packet header = 0x + ffffffff (ts_sec) + ffffffff (ts_nsec) + 00000000 (incl_len) + 00000000 (orig_len)
packet_header = 'f540e051872a0100e0040000e0040000'                                                      # 16 bytes = 32 characters (once at start of every packet)
# Velodyne UDP header
udp_header = 'ffffffffffff60768820118c0800450004d200004000ff11b4a9c0a801c9ffffffff0940094004be0000'     # 42 bytes = 84 characters (once at start of every packet)

bitout = None
total_packets = 0

def init(filename):
    global global_header
    global bitout

    bitout = open(filename, 'wb')
    bytes = binascii.a2b_hex(global_header)
    bitout.write(bytes)

def signal_handler(sig, frame):
    global global_header
    global bitout

    bitout.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def callback(ros_data):
    global packet_header
    global udp_header
    global bitout
    global total_packets

    udp_packets = ''

    num_packets = len(ros_data.packets)
    for i in range(0, num_packets):         # keep adding packet header, udp header and data
        packet_secs = ros_data.packets[i].stamp.secs
        hex_secs = format(packet_secs, '08x')
        packet_nsecs = ros_data.packets[i].stamp.nsecs
        hex_nsecs = format(packet_nsecs, '08x')
        hexdata = binascii.hexlify(ros_data.packets[i].data)
        new_data = hex_secs + hex_nsecs + packet_header[16:]
        new_data += udp_header
        new_data += hexdata.decode('ascii')
        udp_packets += new_data

        total_packets += 1
    #print('Wrote ' + str(total_packets) + ' packets.')

    bytes = binascii.a2b_hex(udp_packets)
    bitout.write(bytes)


# init
if len(sys.argv) < 2:
    print("Not enough arguments!")
    print("Usage: rosbag_to_pcap.py <options> folder_path")

if "-h" in sys.argv:
    print("Usage: rosbag_to_pcap.py <options> out_folder_path")
    print("Script to extract lidar pcap from bag file")
    print(" ")
    print("     -h               print this help message")
    print("     -t               requested topic, default:=/velodyne_packets"  )
    print("     -b               bagfile name")
    exit()

# Path to put finished rosbag into
out_path = sys.argv[-1]
if not os.path.isdir(out_path):
    print("Folder '%s' does not exist" % out_path)
    exit() 

filename = os.path.join(out_path, "lidar.pcap")
velo_topic = "/velodyne_packets"
if "-t" in sys.argv:
    i = sys.argv.index("-t")
    if len(sys.argv) > i+1:
        velo_topic = sys.argv[i+1]

if "-b" in sys.argv:
    i = sys.argv.index("-b")
    if len(sys.argv) > i+1:
        in_bag = rosbag.Bag(sys.argv[i+1])

init(filename)

for topic, msg, t in in_bag.read_messages():
    if topic == velo_topic:
        #print(topic)
        callback(msg)


bitout.close()
#sys.exit(0)
exit()
