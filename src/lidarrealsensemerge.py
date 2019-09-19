#!/usr/bin/env python

import rospy, tf
import math
from sensor_msgs.msg import LaserScan

"""
scan_cam:
header: 
  seq: 802
  stamp: 
    secs: 1565150605
    nsecs: 374808073
  frame_id: "camera_depth_frame"
angle_min: -0.480906009674
angle_max: 0.451981693506
angle_increment: 0.00145991810132
time_increment: 0.0
scan_time: 0.0329999998212
range_min: 0.449999988079
range_max: 3.0
ranges: [   length = 640

$ rostopic echo /camera/depth/camera_info
header: 
  seq: 569
  stamp: 
    secs: 1565200346
    nsecs: 229034185
  frame_id: "camera_depth_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [634.3914794921875, 0.0, 307.9980163574219, 0.0, 634.3914794921875, 240.6433563232422, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [634.3914794921875, 0.0, 307.9980163574219, 0.0, 0.0, 634.3914794921875, 240.6433563232422, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False



scan_lidar:
header: 
  seq: 355
  stamp: 
    secs: 1565148019
    nsecs: 125879953
  frame_id: "laser_frame"
angle_min: 0.0
angle_max: 6.11640310287
angle_increment: 0.0263638067991
time_increment: 0.00139800005127
scan_time: 0.325733989477
range_min: 0.5
range_max: 40.0
ranges: [ length = 233

save 1st cam scan since last lidar scan, and last
use 1/2 of 1st scan for 0-32.5 deg, 1/2 of 2nd scan for 327.5-360 

scan: (merged by ira_laser_tools)
header: 
  seq: 53
  stamp: 
    secs: 1565195917
    nsecs: 176195098
  frame_id: "base_link"
angle_min: -3.1400001049
angle_max: 3.1400001049
angle_increment: 0.00579999992624
time_increment: 0.0
scan_time: 0.333330005407
range_min: 0.449999988079
range_max: 25.0
ranges: [  length = 1083
"""

camscans = []
camrangeslength = -1 	# should be constant, set once only
lidarrangeslength = -1 	# should be constant, set once only 
camanglestart = 0		# should be constant, set once only 
ANGINC = 0.006   		# 0.344 deg
NUMPOINTS = int((math.pi*2)/ANGINC)  # 1047


def cleanup():
	pass
	
	
def mergescans(lidarscandata):
	global camscans
	
	mergedscan = LaserScan()
	mergedscan.header.stamp = lidarscandata.header.stamp
	mergedscan.header.frame_id = 'laser_frame'
	mergedscan.angle_min = 0.0
	mergedscan.angle_max = ANGINC * NUMPOINTS
	mergedscan.angle_increment = ANGINC
	mergedscan.time_increment = 0.0 
	mergedscan.scan_time = lidarscandata.scan_time
	mergedscan.range_min = lidarscandata.range_min
	mergedscan.range_max = lidarscandata.range_max
	mergedscan.ranges=[]
	
	angle = 0.0
	while angle <= mergedscan.angle_max:
		
		distance = 0.0
		distance = getlidarscan(angle, lidarscandata)
		distcam = getcamscan(angle)
		if not distcam == 0.0:
			distance = distcam
			
		mergedscan.ranges.append(distance)
		angle += ANGINC

	scan_pub.publish(mergedscan)
	camscans = []
	
	
def getlidarscan(angle, scan):
	
	""" comp for lidar header rotation """
	#  angle = angle + (camanglestart - math.pi*2)
	#  if angle >= math.pi*2:
		#  angle = angle - math.pi*2
	
	if angle <= scan.angle_max:
		lidarindex = int(angle/scan.angle_increment)
		lidarangle = lidarindex * scan.angle_increment
		if abs(angle - lidarangle) < ANGINC * 0.666:  # <= ANGINC/2:
			return scan.ranges[lidarindex]
			
	return 0
	
	
def getcamscan(angle):
	
	if angle <= camscans[0].angle_max:
		indexangle = angle - camscans[0].angle_min
		#  scannum = 1 # use 2nd scan to match time, when rotated
		scannum = 0

	elif angle >= camanglestart:
		indexangle = angle - (math.pi*2 + camscans[0].angle_min)
		#  scannum = 0 # use 1st scan, when rotate
		scannum = len(camscans)-1 # use last scan
				
	else:
		return 0.0

	index = int(round( (indexangle/(camscans[0].angle_max - camscans[0].angle_min)) * camrangeslength ))
	#  print "index: "+str(index)+", camrangeslength: "+ str(camrangeslength)
	
	if index >= camrangeslength:
		return 0.0
		
	distance = camscans[scannum].ranges[index]
	if math.isnan(distance):  # cam non readings are nan
		distance = 0.0

	return distance

	
def lidarScanCallback(data):
	global lidarrangeslength
	
	if camrangeslength == -1: # no cam data yet, wait
		return
	
	if lidarrangeslength == -1: # set once
		lidarrangeslength = len(data.ranges)
		
	mergescans(data)

	
def camScanCallback(data):
	global camscans, camrangeslength, camanglestart
	
	if camrangeslength == -1: # set once
		camrangeslength = len(data.ranges)
		camanglestart = math.pi*2 + data.angle_min

	camscans.append(data)

	
# main	
rospy.init_node('lidarrealsensemerge', anonymous=False)
rospy.on_shutdown(cleanup)
rospy.loginfo("lidarrealsensemerge init")
scan_pub = rospy.Publisher(rospy.get_param('~merged_scan_topic', 'scan'), LaserScan, queue_size=3)
rospy.Subscriber(rospy.get_param('~lidar_scan_topic', 'scan_lidar'), LaserScan, lidarScanCallback) 
rospy.Subscriber(rospy.get_param('~camera_scan_topic', 'scan_cam'), LaserScan, camScanCallback) 

rospy.spin()
