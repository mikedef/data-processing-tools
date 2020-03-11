import csv
import matplotlib
import numpy
import math
import os

in1_csv = 0
in2_csv = 0
csv_list = []

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
        csv_list.append(sys.argv[i+1]

if "-in2" in sys.argv:
    i = sys.argv.index("-in2")
    if len(sys.argv) > i+1:
        #in2_csv = sys.argv[i+1]
        csv_list.append(sys.argv[i+1]



for csv in csv_list:
    with open(csv, newline='') as f:
        reader = csv.reader(f)
        for line in reader:
            try:
                print (line)
                exit()
