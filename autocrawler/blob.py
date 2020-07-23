#!/usr/bin/env python3

import rclpy 


def main(args=None):
	rclpy.init(args=args)
	node = rclpy.create_node('blob')
	
	
	print("ready")
	node.get_logger().info("node ready");

	rclpy.spin(node)
			
	node.get_logger().info("node Exiting");


if __name__ == '__main__':
	main()
