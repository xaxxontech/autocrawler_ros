#!/usr/bin/env python3

import rclpy 
import autocrawler.oculusprimesocket as oculusprimesocket
from std_msgs.msg import String

node = None

def callback(msg):
	if (msg.data == "disconnected"):
		oculusprimesocket.sendString("driverexit")  
	
	node.get_logger().info("webrtcstatus: "+ msg.data);


def main(args=None):
	global node
	
	rclpy.init(args=args)
	node = rclpy.create_node('webrtc_status_listener')
	node.subscription = node.create_subscription(
		String, 'webrtcstatus', callback, 10)
            
	oculusprimesocket.connect()	
	oculusprimesocket.sendString("log webrtc_status_listener.py connected")  
	
	node.get_logger().info("webrtc_status_listener ready");

	rclpy.spin(node)
			
	node.get_logger().info("webrtc_status_listener Exiting");
    rclpy.shutdown()


if __name__ == '__main__':
	main()
