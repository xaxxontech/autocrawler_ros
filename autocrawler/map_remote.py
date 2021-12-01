#!/usr/bin/env python3

import rclpy
import os, struct, time
from nav_msgs.msg import OccupancyGrid
import autocrawler.oculusprimesocket as oculusprimesocket
import tf_transformations


FRAMEFILEPATH ="/run/shm/map.raw"
LOCKFILEPATH = FRAMEFILEPATH + ".lock"
node = None
scansubscriber = None
firstmap = True


def mapcallBack(data):
	global firstmap
	
	if firstmap:
		firstmap = False
		oculusprimesocket.sendString("state navsystemstatus mapping") 

	if os.path.exists(LOCKFILEPATH):
		return
	
	open(LOCKFILEPATH, 'w') # creates lockfile
	 
	framefile = open(FRAMEFILEPATH, 'wb')
	packeddata = struct.pack('%sb' %len(data.data), *data.data)
	framefile.write(packeddata)
	framefile.close()

	if os.path.exists(LOCKFILEPATH):
		os.remove(LOCKFILEPATH)

	quaternion = ( data.info.origin.orientation.x, data.info.origin.orientation.y,
	data.info.origin.orientation.z, data.info.origin.orientation.w )
	th = tf_transformations.euler_from_quaternion(quaternion)[2]
		
	# width height res originx originy originth updatetime	
	s = "state rosmapinfo "+str(data.info.width)+","+str(data.info.height)+","
	s += str(data.info.resolution)+","+str(data.info.origin.position.x)+","
	s += str(data.info.origin.position.y)+","+str(th)+","+str(time.time())
	
	oculusprimesocket.sendString(s)
	oculusprimesocket.sendString("state rosmapupdated true")


def main(args=None):
	global node
	
	rclpy.init(args=args)
	node = rclpy.create_node('map_remote')

	oculusprimesocket.connect()	
	oculusprimesocket.sendString("state delete roscurrentgoal")
	oculusprimesocket.sendString("state delete rosamcl")
	oculusprimesocket.sendString("state delete rosglobalpath")
	oculusprimesocket.sendString("state delete rosmapinfo")
	oculusprimesocket.sendString("state delete rosscan")
	oculusprimesocket.sendString("state delete rosmapupdated")

	if os.path.exists(LOCKFILEPATH):
		os.remove(LOCKFILEPATH)

	node.subscription = node.create_subscription(OccupancyGrid, "map", mapcallBack, 10)

	try:
		rclpy.spin(node)
	except KeyboardInterrupt: 
		pass 

	
if __name__ == '__main__':
	main()
		
