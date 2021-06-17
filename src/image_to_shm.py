#!/usr/bin/env python3

import rospy
import os, shutil
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

def cleanup():
	rospy.sleep(3)
	deletefiles()
	rospy.loginfo("image_to_shm quit")

		
def deletefiles():
	if os.path.isdir(PATH):
		try:
			shutil.rmtree(PATH)
		except:
			pass
	

def imgCallBack(data):
	global filenum, imgnum
	
	#  imgnum += 1
	#  if data.width > 640 and imgnum %2 == 0: # skip every other high res frame
		#  return 
	
	oldfile = PATH+"/"+str(filenum-MAXFILES)

	if os.path.exists(oldfile):
		os.remove(oldfile)
	
	f = open(PATH+"/"+str(filenum), "wb")
	f.write(data.data)
	f.close()

	filenum += 1
	

deletefiles() 
os.mkdir(PATH)

rospy.init_node('image_to_shm', anonymous=False)
rospy.loginfo("image_to_shm init")
rospy.Subscriber(rospy.get_param('~camera_topic', 'camera/color/image_raw'), Image, imgCallBack) 
rospy.on_shutdown(cleanup)
print ("using topic: "+rospy.get_param('~camera_topic', 'camera/color/image_raw'))

rospy.spin()
