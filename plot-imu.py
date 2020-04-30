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
roll = []
pitch = []
r_mod = []
p_mod = []
y_mod = []
yaw = []


if "-h" in sys.argv:
    print("Usage: python plot-imu.py <options> out_path")
    print("Plotting imu vs time from a csv")
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
        csv_list.append(sys.argv[i+1])

if "-in2" in sys.argv:
    i = sys.argv.index("-in2")
    if len(sys.argv) > i+1:
        csv_list.append(sys.argv[i+1])

print (csv_list)
for imu_csv in csv_list:
    with open(imu_csv, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        # ['time','roll','pitch','yaw']
        for line in reader:
            print(line)
            time.append(line[0])
            
            roll.append(line[1])
            
            pitch.append(line[2])
            
            yaw.append(line[3])

            r_mod.append(line[4])
            p_mod.append(line[5])
            y_mod.append(line[6])
            
plt.plot(time, roll, label='roll', c='r')
plt.plot(time, pitch, label='pitch', c='b')
plt.plot(time, yaw, label='yaw', c='g')
plt.plot(time, r_mod, label='r_mod')
plt.plot(time, p_mod, label='p_mod')
plt.plot(time, y_mod, label='y_mod', c='y')

#plt.plot(time, ekf_y, label='ekf_y', c='y')
plt.xlabel("Time ")
plt.ylabel("imu ")
plt.legend()
plt.show()

#plt.plot(gps_x, gps_y, label='gps')
#plt.plot(ekf_x, ekf_y, label='ekf')
#plt.legend()
#plt.show()
                
