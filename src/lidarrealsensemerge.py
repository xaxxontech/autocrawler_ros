#!/usr/bin/env python

import rospy, tf
import math
from sensor_msgs.msg import LaserScan


camscans = []
camrangeslength = -1 	# should be constant, set once only
lidarrangeslength = -1 	# should be constant, set once only 
camanglestart = 0		# should be constant, set once only 
ANGINC = 0.012   		#  0.006 0.344 deg
NUMPOINTS = int((math.pi*2)/ANGINC)  # 523
scansmerged = False
CAMCOMP = 1.08 			# scale cam points to match <=2m lidar points


def cleanup():
	pass
	
	
def mergescans(lidarscandata):
	global camscans, scansmerged
	
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
	camscans=[]
	
	
def getlidarscan(angle, scan):
	
	""" comp for lidar header rotation """
	#  angle = angle + (camanglestart - math.pi*2)
	#  if angle >= math.pi*2:
		#  angle = angle - math.pi*2
	
	if angle <= scan.angle_max:
		lidarindex = int(angle/scan.angle_increment)
		lidarangle = lidarindex * scan.angle_increment
		if abs(angle - lidarangle) < ANGINC:  # <= ANGINC*0.666:
			return scan.ranges[lidarindex]
			
	return 0
	
	
def getcamscan(angle):
	
	if angle <= camscans[0].angle_max:
		indexangle = angle - camscans[0].angle_min
		#  scannum = 1 # use early scan to match time, when rotated
		scannum = 0

	elif angle >= camanglestart:
		indexangle = angle - (math.pi*2 + camscans[0].angle_min)
		#  scannum = 0 # use 1st scan, when rotate
		scannum = len(camscans)-2 # use later scan to match time
				
	else:
		return 0.0

	index = int(round( (indexangle/(camscans[0].angle_max - camscans[0].angle_min)) * camrangeslength ))
	#  print "index: "+str(index)+", camrangeslength: "+ str(camrangeslength)
	
	if index >= camrangeslength:
		return 0.0
		
	distance = camscans[scannum].ranges[index]
	if math.isnan(distance):  # cam non readings are nan
		distance = 0.0
		
	distance = distance * CAMCOMP

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
