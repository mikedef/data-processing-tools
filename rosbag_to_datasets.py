#!/usr/bin/env python
import os
import sys
import subprocess
import glob
import shutil

OUTBAG_NAME = "section.bag"

if len(sys.argv) < 2:
  print ("Usage: " + sys.argv[0] + " <filename>\n")
  exit()

if "-h" in sys.argv:
  print ("Usage: bagsToDatasets.py <filename>")
  print ("Script iterates through folders in <filename> and creates datasets")
  print ("Each line in <filename>:    <path to input bag file>, <time start>, <time end>, <path to output bag folder>")
  print ("   -h          print this help message")
  exit()

folder_file = sys.argv[1]
if not os.path.isfile(folder_file):
  print("file does not exist: ", folder_file)
  exit()
  
# get a list of LOG folders to walk  
folder_list = []
with open(folder_file, 'r') as dir_file:
  folder_list = [line.rstrip('\n') for line in dir_file.readlines()]
dir_file.close()  
   
# walk the folder list
for f in folder_list:
  if f == '' or f.startswith('#'):
    continue
  print ("Processing: " + f)  

  try: 
    args = f.split(",") 
    if (len(args) != 4):
      print("bad bagfile line:" + f)
      continue
    inbag = args[0]
    start = args[1]
    end = args[2]
    bagdir = args[3]
    outbag = os.path.join(bagdir, OUTBAG_NAME)
    
    if (not os.path.isfile(inbag)):
      print("bagfile missing:" + inbag)
      continue      
    
    if (not os.path.isdir(bagdir)):
      os.mkdir(bagdir)

    if os.path.isfile(outbag):
      os.remove(outbag)
    print("filtering master")      
    subprocess.call(["rosbag", "filter", inbag, outbag, "t.secs >= " + start + " and t.secs <= " + end])  
    print("extracting images")
    subprocess.call(["python", "extract_images2.py", "-b", outbag, "-lc", "-cc", "-rc", "-ir", "-r0", "-r1", bagdir])
    print("extracting lidar pcap")
    subprocess.call(["python", "rosbag_to_pcap.py", "-b", outbag, bagdir])
    print("extracting key vars to csv")
    subprocess.call(["python", "rosbag_to_csv.py", "-imu", "-lv", "-gps", "-rad", "-b", outbag, bagdir])
    
    # consolidate files and videos
    csv_folder = os.path.join(bagdir, "csv")
    if os.path.isdir(csv_folder):
      shutil.rmtree(csv_folder)
    os.mkdir(csv_folder)
    for file in glob.glob(os.path.join(bagdir, "*.csv")):
      shutil.move(file, csv_folder)
    
    video_folder = os.path.join(bagdir, "videos")
    if os.path.isdir(video_folder):
      shutil.rmtree(video_folder)
    os.mkdir(video_folder)
    for folder in ['center_camera', 'ir_camera', 'left_camera', 'RADRAW0', 'RADRAW1', 'right_camera']: 
      for file in glob.glob(os.path.join(os.path.join(bagdir, folder), '*.mp4')):
        shutil.move(file, video_folder)

    meta_folder = os.path.join(bagdir, "metadata")
    if not os.path.isdir(meta_folder):
      os.mkdir(meta_folder)

  except:
    print("Error during postprocessing of ", f)
    print(sys.exc_info()[0].__name__)
    print(sys.exc_info()[1])
    print(sys.exc_info()[2])
      
print("bagsToDatasets.py done")
