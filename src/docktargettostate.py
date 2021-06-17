#!/usr/bin/env python3

import rospy, tf
from fiducial_msgs.msg import FiducialTransformArray 
import oculusprimesocket
import math

dockfound = False
dockcamready = False

def cleanup():
	oculusprimesocket.sendString("state delete dockfound") 
	oculusprimesocket.sendString("state delete dockcamready") 	
	oculusprimesocket.sendString("log docktargettostate.py disconnecting")  # goodbye 
	
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
	# camangle = math.atan(xc/zc)
	baselinkangle = math.atan((xc+ybasecam)/(zc+xbasecam)) # target left,  should be positive
	baselinkdistance = math.sqrt((xc+ybasecam)**2 + (zc+xbasecam)**2)
	
	quaternion = (data.transforms[0].transform.rotation.x,
					data.transforms[0].transform.rotation.y,
					data.transforms[0].transform.rotation.z,
					data.transforms[0].transform.rotation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	targetpitch = euler[1] # TODO: this is to dockcam, should be to base_link
		
	# baselinkdistance baselinkangle camangle targetpitch
	# dockmetrics = str(baselinkdistance)+" "+str(math.degrees(baselinkangle))+" "+str(math.degrees(camangle))+" "+str(math.degrees(targetpitch))
	dockmetrics = "{0:.4f}".format(baselinkdistance) + " "
	dockmetrics += "{0:.3f}".format(math.degrees(baselinkangle)) + " "
	# dockmetrics += "{0:.3f}".format(math.degrees(camangle)) + " "
	dockmetrics += "{0:.3f}".format(math.degrees(targetpitch))
	oculusprimesocket.sendString("state dockmetrics "+dockmetrics)
	oculusprimesocket.sendString("state dockfound true")
	dockfound = True

	
# main

rospy.init_node('docktargettostate', anonymous=False)
rospy.on_shutdown(cleanup)
oculusprimesocket.connect()
oculusprimesocket.sendString("docktargettostate.py connected")  
rospy.Subscriber("fiducial_transforms", FiducialTransformArray, transformCallback)
oculusprimesocket.sendString("state delete dockfound") 
oculusprimesocket.sendString("state dockcamready false") 

xbasecam = None
ybasecam = None

listener = tf.TransformListener()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
	
	if not xbasecam:
		try:
			(trans,rot) = listener.lookupTransform('/base_link', '/dockcam', rospy.Time(0))
			xbasecam=-trans[0] # negate because reversed
			ybasecam=-trans[1] # negate because reversed
			# dockcamangle = math.atan(y/x) # 0.0191059551184
			# print("xbasecam: "+str(xbasecam)+" ybasecame: "+str(ybasecam))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# print("tf lookup error")
			rate.sleep()
			continue

	rate.sleep()
