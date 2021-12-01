#!/usr/bin/env python3

"""
connect to xaxxon robot java application
poll server for odometry data
broadcast tranform between base_link and odom frames
"""

import rclpy 
import autocrawler.oculusprimesocket as oculusprimesocket
import tf_transformations
from math import radians, sin, cos
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import time

pos = [0.0, 0.0, 0.0]
before = 0
now = 0
LAG = 0.0 #0.035  
node = None
br = None
odom_pub = None


def broadcast(s):
	global before, pos, now 
	
	now = node.get_clock().now() - rclpy.time.Duration(seconds=LAG)
	dt = (now-before).nanoseconds/1e9
	before = now

	distance = float(s[2])/1000
	delta_x = distance * cos(pos[2])
	delta_y = distance * sin(pos[2]) 
	delta_th = radians(float(s[3]))
	pos[0] += delta_x
	pos[1] += delta_y
	pos[2] += delta_th
	
	#tf
	t = TransformStamped()

	t.header.stamp = now.to_msg()
	t.header.frame_id = 'odom'
	t.child_frame_id = 'base_link'

	t.transform.translation.x = pos[0]
	t.transform.translation.y = pos[1]
	t.transform.translation.z = 0.0

	q = tf_transformations.quaternion_from_euler(0, 0, pos[2])
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	br.sendTransform(t)
	
	# odom
	o = Odometry()
	o.header.stamp = now.to_msg()
	o.header.frame_id = "odom"
	o.child_frame_id = "base_link"

	#set the position
	o.pose.pose.position.x = pos[0]
	o.pose.pose.position.y = pos[1]
	o.pose.pose.position.z = 0.0

	o.pose.pose.orientation.x = q[0]
	o.pose.pose.orientation.y = q[1]
	o.pose.pose.orientation.z = q[2]
	o.pose.pose.orientation.w = q[3]

	#set the velocity
	o.twist.twist.linear.x = delta_x / dt
	o.twist.twist.linear.y = delta_y / dt
	o.twist.twist.linear.z = 0.0
	o.twist.twist.angular.x = 0.0
	o.twist.twist.angular.y = 0.0
	o.twist.twist.angular.z = delta_th / dt
	
	#publish
	odom_pub.publish(o)


def cleanup():
	oculusprimesocket.sendString("odometrystop")
	oculusprimesocket.sendString("log odom_tf.py disconnecting") 

	
def main(args=None):
	global node, before, br, odom_pub
	
	rclpy.init(args=args)
	node = rclpy.create_node('odom_tf')
	before = node.get_clock().now()
	br = TransformBroadcaster(node)
	odom_pub = node.create_publisher(Odometry, 'odom', 10)
            
	oculusprimesocket.connect()	
	oculusprimesocket.sendString("log odom_tf.py connected")  
	oculusprimesocket.sendString("odometrystart")
	broadcast("* * 0 0".split()) # broadcast zero odometry baseline
		
	while True:
		s = oculusprimesocket.waitForReplySearch("<state> distanceangle ")
		if not s=="":
			broadcast(s.split())

		try:
			rclpy.spin_once(node, timeout_sec=0.01)
		except KeyboardInterrupt: 
			cleanup() 
			break


if __name__ == '__main__':
	main()

