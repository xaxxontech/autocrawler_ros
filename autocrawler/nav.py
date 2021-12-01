#!/usr/bin/env python3

import math, time
import rclpy
import autocrawler.oculusprimesocket as oculusprimesocket
from geometry_msgs.msg import Twist

node = None
lastlinear = lastangular = lastmove = None
lastmovetime = 0

def twistCallback(data):

	linear = data.linear.x
	angular = data.angular.z
	
	move(linear, angular)
	
	
def move(linear, angular):
	global lastmove, lastmovetime
	# global lastlinear, lastangular

	# don't send repeat stop commands
	# if linear == lastlinear and angular == lastangular and linear == 0 and angular == 0:
		# return

	# lastlinear = linear
	# lastangular = angular	
	
	t = time.time()
	if t - lastmovetime < 1.0:
		return

	cmd = None
	
	d = "0.3" # .3
	a = "60.0" # 25
	arcmult = 3 # 3
	
	if abs(angular) >= 0.8:
		linear = 0
	
	if abs(angular) < 0.1:
		angular = 0

	if linear == 0 and angular == 0:
		cmd = "move stop"
		
	elif linear > 0 and angular == 0:
		#  cmd = "forward "+d
		cmd = "forward "+d
		
	elif linear < 0 and angular == 0:
		cmd = "backward "+d
		
	elif linear == 0 and angular > 0:
		cmd = "left "+int(a*float(angular))
		
	elif linear == 0 and angular < 0:
		cmd = "right "+int(a*float(angular))
		
	elif linear > 0 and not angular == 0:  # forward arc
		angle = str(int(math.degrees(angular))/arcmult)
		cmd = "arcmove "+d+" "+angle
	
	elif linear < 0 and not angular == 0:  # backwards arc
		angle = str(int(math.degrees(angular))/arcmult)
		cmd = "arcmove -"+d+" "+angle
	
	# if cmd == lastmove:
		# return
				
	if not cmd == None:
		oculusprimesocket.sendString(cmd)
		lastmove = cmd
		lastmovetime = t
		print (str(linear)+", "+str(angular)+", "+cmd)
		


def main(args=None):
	global node
	
	oculusprimesocket.connect()
	
	rclpy.init(args=args)
	node = rclpy.create_node('autocrawler_nav')
	node.create_subscription(Twist, "cmd_vel", twistCallback, 10)

	try:
		rclpy.spin(node)
	except KeyboardInterrupt: 
		pass 

	
if __name__ == '__main__':
	main()
		
