#!/usr/bin/env python3


# TODO: add dockoffset param

"""
/fiducial_transforms:
---
header:
  stamp:
    sec: 1636005345
    nanosec: 246740322
  frame_id: usb_cam
image_seq: 0
transforms:
- fiducial_id: 17
  transform:
    translation:
      x: 0.006667401614500212
      y: 0.017128746437704137
      z: 0.31425369096485023
    rotation:
      x: 0.9998274933858048
      y: -0.011925586625090056
      z: -0.004901192754580819
      w: 0.01336944887003686
  image_error: 0.027948993782047182
  object_error: 0.0010660282325613512
  fiducial_area: 8057.8183737979225
---
"""


import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from fiducial_msgs.msg import FiducialTransformArray 
import autocrawler.oculusprimesocket as oculusprimesocket
import math


dockfound = False
dockcamready = False
node = None
xbasecam = None
ybasecam = None

# from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/ 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        

def transformCallback(data):
	global dockfound, dockcamready
	
	if not xbasecam:
		return
		
	if not dockcamready:
		dockcamready = True
		oculusprimesocket.sendString("state dockcamready true")
		
	if len(data.transforms)==0: #no target found
		if dockfound == True: # avoid unnecessary socket chatter
			oculusprimesocket.sendString("state dockfound false") 
			dockfound = False
		return

	xc = -(data.transforms[0].transform.translation.x + 0.018) # add bot/dock contacts offset actual is 0.024
	zc = data.transforms[0].transform.translation.z

	baselinkangle = math.atan((xc+ybasecam)/(zc+xbasecam)) # target left,  should be positive
	baselinkdistance = math.sqrt((xc+ybasecam)**2 + (zc+xbasecam)**2)
	
	quaternion = (data.transforms[0].transform.rotation.x,
					data.transforms[0].transform.rotation.y,
					data.transforms[0].transform.rotation.z,
					data.transforms[0].transform.rotation.w)
	euler = euler_from_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
	targetpitch = euler[1] # TODO: this is to dockcam, should be to base_link
		
	dockmetrics = "{0:.4f}".format(baselinkdistance) + " "
	dockmetrics += "{0:.3f}".format(math.degrees(baselinkangle)) + " "
	dockmetrics += "{0:.3f}".format(math.degrees(targetpitch))
	oculusprimesocket.sendString("state dockmetrics "+dockmetrics)
	oculusprimesocket.sendString("state dockfound true")
	dockfound = True

	
def main(args=None):
	global node, xbasecam, ybasecam
	
	rclpy.init(args=args)
	node = rclpy.create_node('docktargettostate')
	node.subscription = node.create_subscription(
		FiducialTransformArray, '/fiducial_transforms', transformCallback, 10)
	
	oculusprimesocket.connect()
	oculusprimesocket.sendString("docktargettostate.py connected")  
	oculusprimesocket.sendString("state delete dockfound") 
	oculusprimesocket.sendString("state dockcamready false") 
	
	tf_buffer = Buffer()
	listener = TransformListener(tf_buffer, node)
	
	while rclpy.ok():
		# rclpy.sleep(0.01)
		rclpy.spin_once(node, timeout_sec=0.01)
		
		if not xbasecam:
			try:
				trans = tf_buffer.lookup_transform('base_link', 'dockcam', rclpy.time.Time())
				xbasecam=-trans.transform.translation.x # negate because img reversed
				ybasecam=-trans.transform.translation.y # negate because img reversed
				print ("XBASECAM found: "+str(xbasecam))

			except TransformException:
				continue
				
		
		
	#cleanup 
	oculusprimesocket.sendString("state delete dockfound") 
	oculusprimesocket.sendString("state delete dockcamready") 	
	oculusprimesocket.sendString("log docktargettostate.py disconnecting")  # goodbye 
		
if __name__ == '__main__':
	main()

