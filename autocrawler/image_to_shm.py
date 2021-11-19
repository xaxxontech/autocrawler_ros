#!/usr/bin/env python3

import rclpy
import os, shutil, time
import subprocess
from sensor_msgs.msg import Image

'''
rostopic info /camera/color/image_raw
Type: sensor_msgs/Image

rosmsg show sensor_msgs/Image
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data


$ rostopic echo /camera/color/image_raw
header: 
  seq: 1097
  stamp: 
    secs: 1569276117
    nsecs: 297078848
  frame_id: "camera_color_optical_frame"
height: 480
width: 640
encoding: "rgb8"
is_bigendian: 0
step: 1920
data: [... ...]

'''


filenum = 0
MAXFILES = 100
PATH = "/dev/shm/rosimgframes"  # filename format: frame number (integer) only, with no extension
imgnum = 0
node = None


def cleanup():
	time.sleep(3)
	deletefiles()
	node.get_logger().info("image_to_shm exiting")

		
def deletefiles():
	if os.path.isdir(PATH):
		try:
			shutil.rmtree(PATH)
		except:
			pass
	

def imgCallBack(data):
	global filenum, imgnum
	
	oldfile = PATH+"/"+str(filenum-MAXFILES)

	if os.path.exists(oldfile):
		os.remove(oldfile)
	
	f = open(PATH+"/"+str(filenum), "wb")
	f.write(data.data)
	f.close()

	filenum += 1
	
	
def main(args=None):
	global node
	
	deletefiles() 
	os.mkdir(PATH)

	rclpy.init(args=args)
	node = rclpy.create_node('image_to_shm')
	node.get_logger().info("image_to_shm ready");
	
	result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE)
	topics = result.stdout.decode('utf-8').split()
	
	# print (topics)
	# ros2 realsense: /color/image_raw
	# ros2 dockcam: /image_raw
		
	topic = None
	for t in topics:
		if t.endswith('image_raw'):
			topic = t
			break
		
	if not topic:
		node.get_logger().error("no image_raw topic found")
		cleanup()
		return
	

	node.get_logger().info("using topic: "+topic)

	#Type: sensor_msgs/msg/Image
	node.subscription = node.create_subscription(Image, topic, imgCallBack, 10)

	try:
		rclpy.spin(node)
	except KeyboardInterrupt: 
		cleanup()  




if __name__ == '__main__':
	main()
