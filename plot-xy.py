import csv
from matplotlib import pyplot as plt
import numpy
import math
import os
import sys

in1_csv = 0
in2_csv = 0
csv_list = []
time = []
ekf_x = []
ekf_y = []
gps_x = []
gps_y = []

if "-h" in sys.argv:
    print("Usage: python plot-xy.py <options> out_path")
    print("Plotting xy vs time from a csv")
    print("     -in1      path to csv1")
    print("     -in2      path to csv2")
    exit()

out_path = sys.argv[-1]
if not os.path.isdir(out_path):
    print("Folder '%s' does not exist" % out_path)
    exit()

if "-in1" in sys.argv:
    i = sys.argv.index("-in1")
    if len(sys.argv) > i+1:
        #in1_csv = sys.argv[i+1]
        csv_list.append(sys.argv[i+1])

if "-in2" in sys.argv:
    i = sys.argv.index("-in2")
    if len(sys.argv) > i+1:
        #in2_csv = sys.argv[i+1]
        csv_list.append(sys.argv[i+1])

print (csv_list)
for xy_csv in csv_list:
    with open(xy_csv, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        # ['time', 'gps_time', 'gps_x', 'gps_y', 'ekf_time', 'ekf_x', 'ekf_y']
        for line in reader:
            print(line)
            time.append(line[0])
            
            gps_x.append(line[2])
            
            gps_y.append(line[3])
            
            ekf_x.append(line[5])
            
            ekf_y.append(line[6])
            
plt.plot(time, gps_x, label='gps_x', c='r')
plt.plot(time, gps_y, label='gps_y', c='b')
plt.plot(time, ekf_x, label='ekf_x', c='g')
plt.plot(time, ekf_y, label='ekf_y', c='y')
plt.xlabel("Time ")
plt.ylabel("x-y ")
plt.legend()
plt.show()

plt.plot(gps_x, gps_y, label='gps')
plt.plot(ekf_x, ekf_y, label='ekf')
plt.legend()
plt.show()
                
