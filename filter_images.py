# 2020-06
# Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
# License: MIT

'''
    Script to extract images from a folder, filter out frames, and save to another folder. 
'''

import os
import sys
import shutil
import subprocess
import glob
import time
import cv2 as cv

#init
ir_images = False

# Helpers
def create_folder(folder):
    os.chdir(parent_directory)  # change into folder where imeges are located
    if os.path.isdir(folder): # if folder is already there remove
        shutil.rmtree(folder)
    try:
        os.mkdir(folder) # if folder is not there make
    except:
        pass

# process command line
if len(sys.argv) < 2:
    print("Not enough arguments!")
    print("Usage: filter_images <options> parent_directory")
    exit()

if "-h" in sys.argv:
    print("Usage: filter_images <options> folder_path")
    print("Last argument is the path to the parent directory where images are located")
    print("")
    print("     -h     print this help message")
    print("     -f     folder to save images to")
    print("     -i     filter out images every x photo and save to folder")
    print("     -ir    to extract ir images")
    print("     -l     label name")
    exit()

parent_directory = sys.argv[-1]
if not os.path.isdir(parent_directory):
    print("Folder '%s' does not exist" % parent_directory)

if "-i" in sys.argv:
    i = sys.argv.index("-i")
    if len(sys.argv) > i+1:
        idx = sys.argv[i+1]

if "-f" in sys.argv:
    i = sys.argv.index("-f")
    if len(sys.argv) > i+1:
        label_folder = sys.argv[i+1]
        create_folder(sys.argv[i+1])

if "-ir" in sys.argv:
    ir_images = True

if "-l" in sys.argv:
    i = sys.argv.index("-l")
    if len(sys.argv) > i+1:
        label_name = sys.argv[i+1]
    
print ("index", idx)
counter = 0

if ir_images:
    
    for file in glob.glob(os.path.join(parent_directory, "ir_camera/*.jpg")):
        #print len(glob.glob(os.path.join(parent_directory, "ir_camera/*.jpg")))
        #print (file)
        time = file.strip(os.path.join(parent_directory, "ir_camera/"))
        time = time.strip(".jpg")
        #print (time)
        #exit()

        counter += 1
        
        if counter >= int(idx):
            
            img = cv.imread(file)
            #cv.imshow('test', img)
            #cv.waitKey(0)
            #cv.destroyAllWindows()

            if (os.path.isdir(label_folder)):
                cv.imwrite(os.path.join(label_folder, label_name + "_" + time + ".jpg"), img)
                counter = 0
                
exit()


          
