#!/usr/bin/env python

# Author: Nick R. Rypkema (rypkema@mit.edu)
# License: MIT

import os
import sys
import re
import math
import numpy as np

# ros
#import ros_numpy
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

RADAR_SCALE_MULT = math.sqrt(2)/10
RADAR_HEADER_SIZE = 8
RADAR_LINE_SIZE = 536
RADAR_NUM_LINES = 32
RADAR_PIXELS_IN_LINE = 512
RADAR_SPOKE_ANGLE = 0
def read_radar_data(f, offset, nbytes):
    global RADAR_SPOKE_ANGLE
    f.seek(offset)
    s = nbytes
    str_data = f.read(s)
    num_rad_lines = (s - RADAR_HEADER_SIZE)/RADAR_LINE_SIZE
    if num_rad_lines != RADAR_NUM_LINES:
        return None, None
    data = np.frombuffer(str_data, dtype=np.uint8)
    coll_info = np.zeros((RADAR_NUM_LINES, 2))
    coll_data = np.zeros((RADAR_NUM_LINES, RADAR_PIXELS_IN_LINE), dtype=np.uint8)
    prev_angle = 0
    for j in range(RADAR_NUM_LINES):
        # the angle of the beam is recorded as two bytes and ranges [0,4096]
        angle = data[17 + j*536]
        angle = angle << 8
        angle |= data[16 + j*536]
        angle = float(angle)*360.0/4096.0 - 270

        # determine max range based on RadarPi approach
        max_range = 0
        large_range = (data[15 + j*536] << 8) | data[14 + j*536]
        small_range = (data[21 + j*536] << 8) | data[20 + j*536]
        if (large_range) != 0x80:
          max_range = large_range * 64
        else:
          if small_range != 0x8000:
            max_range = small_range / 4
          else:
            # invalid range values
            print("ERROR: cannot determine max_range")
            continue

        start_offset = 32 + j*536
        end_offset = start_offset + RADAR_PIXELS_IN_LINE
        vals = list(data[start_offset:end_offset])
        coll_info[j,0] = angle
        coll_info[j,1] = max_range
        coll_data[j,:] = vals

        if RADAR_SPOKE_ANGLE==0:
            RADAR_SPOKE_ANGLE = angle-prev_angle
        if angle-prev_angle != RADAR_SPOKE_ANGLE:
            RADAR_SPOKE_ANGLE = angle-prev_angle
        prev_angle = angle

    return coll_info, coll_data

def ang_diff(angle1, angle2):
    a = angle1 - angle2
    a = (a + 180) % 360 - 180
    return a

def find_closest(vec, targets):
    idx = vec.searchsorted(targets)
    idx = np.clip(idx, 1, len(vec)-1)
    left = vec[idx-1]
    right = vec[idx]
    idx -= targets - left < right - targets
    return idx

def systematic_resample(num_particles, particles, weights):
    positions = (np.random.uniform() + np.arange(0, num_particles))/num_particles
    idxs = np.zeros([1, num_particles], dtype=np.int)
    cumulative_sum = np.cumsum(weights)
    i_ = 0
    j_ = 0
    while i_ < num_particles:
        if positions[i_] < cumulative_sum[j_]:
            idxs[0,i_] = j_
            i_ += 1
        else:
            j_ += 1
    particles[0,:] = particles[0,idxs]
    particles[1,:] = particles[1,idxs]
    weights = weights[0,idxs]
    weights /= np.sum(weights)
    return particles, weights

def xy_disperse_particles(particles, x_jitter, y_jitter):
    particles[0,:] = np.random.normal(particles[0,:], x_jitter)
    particles[1,:] = np.random.normal(particles[1,:], y_jitter)
    return particles

def particles_to_xyparticles(particles, max_range_val, max_angle_val, min_range, max_range, angle_start, angle_end):
    angdiff = abs(ang_diff(angle_start, angle_end))
    rparticles = (particles[1,:].astype(np.float32)/max_range_val)*(max_range-min_range)+min_range
    aparticles = (particles[0,:].astype(np.float32)/max_angle_val)*(angdiff)+angle_start
    xyparticles = np.zeros((2, particles.shape[1]))
    xyparticles[0,:] = rparticles*np.sin(aparticles*math.pi/180)
    xyparticles[1,:] = rparticles*np.cos(aparticles*math.pi/180)
    return xyparticles

type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

def arr_to_fields(cloud_arr):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    for field_name in cloud_arr.dtype.names:
        np_field_type, field_offset = cloud_arr.dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        pf.count = 1 # is this ever more than one?
        fields.append(pf)
    return fields

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None, merge_rgb=False):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    if merge_rgb:
        cloud_arr = merge_rgb_fields(cloud_arr)

    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = arr_to_fields(cloud_arr)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg

def array_to_xyz_pointcloud2f(cloud_arr, stamp=None, frame_id=None, merge_rgb=False):
    """ convert an Nx3 float array to an xyz point cloud.
    beware of numerical issues when casting from other types to float32.
    """
    cloud_arr = np.asarray(cloud_arr, dtype=np.float32)
    if not cloud_arr.ndim==2: raise ValueError('cloud_arr must be 2D array')
    if not cloud_arr.shape[1]==3: raise ValueError('cloud_arr shape must be Nx3')
    xyz = cloud_arr.view(np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])).squeeze()
    return array_to_pointcloud2(xyz, stamp=stamp, frame_id=frame_id, merge_rgb=merge_rgb)


# process command line
if len(sys.argv) < 3:
    print("Not enough arguments!")
    print("Usage: alog_to_bag_pointcloud_only.py [RADAR0:RADAR1] alog_file")
    exit()


alogfile = str(sys.argv[2])
print(alogfile)

blog_root = alogfile.partition(".alog")[0]
blogfile = blog_root + ".blog"
#blogfile = alogfile[:-4] + 'blog'
print(blogfile)

if (sys.argv[1] == 'RADAR0'):
  params = {'bag_name': '_radar0_pointcloud_only.bag',
            'blog_term': 'RADRAW0  ',
            'bag_term': '/broadband_radar/channel_0/pointcloud'}
else:
  params = {'bag_name': '_radar1_pointcloud_only.bag',
            'blog_term': 'RADRAW1  ',
            'bag_term': '/broadband_radar/channel_1/pointcloud'}
rosbagfile = blog_root + params['bag_name']
print('Creating ' + rosbagfile)
bag = rosbag.Bag(rosbagfile, 'w')

num_particles = 5000

got_time = False
with open(alogfile,'r') as f:
    for line in f:
        if ("DB_TIME" in line):
            if not got_time:
                data = str.split(line)
                time = float(data[0])
                unix_time = float(data[3])
                start_time = unix_time-time
                got_time = True
            else:
                data = line
data = str.split(data)
time = float(data[0])
unix_time = float(data[3])
end_time = unix_time
radar_duration = end_time - start_time

prev_time0 = None
with open(blogfile,'rb') as bf:
    with open(alogfile,'r') as f:
        for line in f:
            if (params['blog_term'] in line) and ("MOOS_BINARY" in line):
                data0 = str.split(line)
                binary0 = str.split(data0[3],',')
                offset0 = str.split(binary0[2],'=')[1]
                nbytes0 = re.split('=|<',binary0[3])[1]
                time0 = float(data0[0])
                nbytes0 = int(nbytes0)
                offset0 = int(offset0)
                info0, d0 = read_radar_data(bf, offset0, nbytes0)

                per_done = (time0)/radar_duration*100
 
                if d0 is not None:
                    if prev_time0 is None:
                        # create new message
                        arc0 = []
                        arcinfo0 = []
                        arc0.append(d0)
                        arcinfo0.append(info0)

                        prev_time0 = time0
                    if time0 != prev_time0:
                        # save radar message
                        arcinfo0 = np.vstack(arcinfo0)
                        arc0 = np.vstack(arc0)
                        arc0_time = prev_time0 + start_time

                        # save radar sampled point cloud message
                        particles_ar = np.zeros((2,num_particles), dtype=np.int16)
                        particles_xy = np.zeros((3,num_particles), dtype=np.float32)
                        particles = np.zeros((num_particles,3), dtype=np.float32)
                        particles_ar[0,:] = np.random.randint(0, arcinfo0.shape[0], size=(1, num_particles), dtype=np.int16)
                        particles_ar[1,:] = np.random.randint(0, arc0.shape[1], size=(1, num_particles), dtype=np.int16)
                        weights_ar = np.ones((1,num_particles))/num_particles
                        weights_ar = weights_ar * arc0[particles_ar[0,:], particles_ar[1,:]]
                        weights_ar = weights_ar + 1e-9
                        weights_ar = weights_ar/np.sum(weights_ar)
                        low_weighted_particles = weights_ar <= (1.0/num_particles + 1e-9)
                        particles_ar[0,low_weighted_particles[0,:]] = 0
                        particles_ar[1,low_weighted_particles[0,:]] = 0
                        particles_ar, weights_ar = systematic_resample(num_particles, particles_ar, weights_ar)
                        particles_xy[0:2,:] = particles_to_xyparticles(particles_ar, arc0.shape[1]-1, arcinfo0.shape[0]-1, 0.0, np.max(arcinfo0[:,1]), arcinfo0[0,0], arcinfo0[-1,0])
                        particles_xy[0:2,:] = xy_disperse_particles(particles_xy[0:2,:], 1, 1)
                        particles[:,0] = particles_xy[0,:]
                        particles[:,1] = particles_xy[1,:]
                        particles[:,2] = particles_xy[2,:]
                        hdr = Header()
                        hdr.stamp.secs = int(arc0_time)
                        hdr.stamp.nsecs = int((arc0_time - int(arc0_time))*1e9)
                        msg_pc0 = array_to_xyz_pointcloud2f(particles, hdr.stamp, 'velodyne')
                        bag.write(params['bag_term'], msg_pc0, rospy.Time(secs=hdr.stamp.secs, nsecs=hdr.stamp.nsecs))

                        arc0 = []
                        arcinfo0 = []
                        arc0.append(d0)
                        arcinfo0.append(info0)

                        prev_time0 = time0
                    else:
                        arc0.append(d0)
                        arcinfo0.append(info0)

bag.close()
