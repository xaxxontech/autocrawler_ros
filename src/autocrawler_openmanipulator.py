#!/usr/bin/env python

"""
communicate between autocrawler java and robotis openmanipulator ros1

$ rosservice type /open_manipulator/goal_joint_space_path | rossrv show
string planning_group
open_manipulator_msgs/JointPosition joint_position
  string[] joint_name
  float64[] position
  float64 max_accelerations_scaling_factor
  float64 max_velocity_scaling_factor
float64 path_time
"""


import rospy, tf
import re, os
import oculusprimesocket
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetActuatorState, SetJointPosition
from open_manipulator_msgs.msg import JointPosition

currentJointPositions = None
recordedJointPositions = [None,None,None]
startupok = False
rest = True

def jointCallback(data):
	global currentJointPositions, startupok
	
	joint_names = []
	positions = []

	i = 0
	while i < len(data.name):
		joint_names.append(data.name[i])
		positions.append(data.position[i])
		# print("joint: "+data.name[i]+", angle: "+str(data.position[i]))
		i += 1
	
	currentJointPositions = JointPosition(joint_names, positions, 0.0, 0.0)
	
	if not startupok:
		startupok = True
		oculusprimesocket.sendString("state arm ready")  
		oculusprimesocket.sendString("messageclients openmanipulator ready")  
	

def printJointPos(jointpos):
	i = 0
	while i < len(currentJointPositions.joint_name):
		print("joint: "+currentJointPositions.joint_name[i]+", angle: "+str(currentJointPositions.position[i]))
		i += 1

def getJointPosition():
	joint_names = []
	positions = []
	i = 0
	while i < len(currentJointPositions.joint_name):
		joint_names.append(currentJointPositions.joint_name[i])
		positions.append(currentJointPositions.position[i])
		i += 1
	return JointPosition(joint_names, positions, currentJointPositions.max_accelerations_scaling_factor, currentJointPositions.max_velocity_scaling_factor)

def moveToFloor():
	global rest
	rospy.wait_for_service('open_manipulator/goal_joint_space_path')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
	newjointpositions = getJointPosition()
	newjointpositions.position[0] = -0.0322135984898
	newjointpositions.position[1] = 1.44500994682
	newjointpositions.position[2] = -0.655009806156
	newjointpositions.position[3] = 0.786932170391
	set_joint_position('', newjointpositions , 2)	
	rest=False
	
def moveToHome():
	global rest
	rospy.wait_for_service('open_manipulator/goal_joint_space_path')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
	newjointpositions = getJointPosition()
	newjointpositions.position[0] = -0.00153398083057
	newjointpositions.position[1] = -1.05231082439
	newjointpositions.position[2] = 0.369689375162
	newjointpositions.position[3] = 0.701029241085
	set_joint_position('', newjointpositions , 2)
	rest=False
	
def gripperopen():
	rospy.wait_for_service('open_manipulator/goal_tool_control')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_tool_control', SetJointPosition)
	newjointpositions = getJointPosition()
	newjointpositions.position[4] = 0.01
	set_joint_position('', newjointpositions , 2)

def gripperclose():
	rospy.wait_for_service('open_manipulator/goal_tool_control')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_tool_control', SetJointPosition)
	newjointpositions = getJointPosition()
	newjointpositions.position[4] = -0.01
	set_joint_position('', newjointpositions , 2)	

def armrest():
	global rest
	rospy.wait_for_service('open_manipulator/goal_joint_space_path')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
	newjointpositions = getJointPosition()
	newjointpositions.position[0] = -0.00306796166115
	newjointpositions.position[1] = -1.56466042995
	newjointpositions.position[2] = 1.45728182793
	newjointpositions.position[3] = 0.506213665009
	set_joint_position('', newjointpositions , 2)
	rest = True

def armForward():
	global rest
	rospy.wait_for_service('open_manipulator/goal_joint_space_path')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
	newjointpositions = getJointPosition()
	newjointpositions.position[0] = -0.00153398083057
	newjointpositions.position[1] = 0.00766990426928
	newjointpositions.position[2] = 0.0291456356645
	newjointpositions.position[3] = 0.00613592332229
	set_joint_position('', newjointpositions , 2)
	rest = False
	
	
def main(args=None):
	global rest
	
	rospy.init_node('autocrawler_openmanipulator', anonymous=False)
	rospy.Subscriber("open_manipulator/joint_states", JointState, jointCallback) 
	
	oculusprimesocket.connect()	
	oculusprimesocket.sendString("log autocrawler_openmanipulator.py connected")  

	# jointpos = JointPosition(["joint1","joint2","joint3","joint4","gripper"],[0,0,0,0],1,1)

	while not rospy.is_shutdown():
		s = oculusprimesocket.replyBufferSearch("<state> arm")
		
		if re.search("arm on", s): # enable
			oculusprimesocket.sendString("malgcommand W")
			# TODO: send open_manipulator_controller roslaunch command 
			
		elif re.search("arm off", s):
			print("off")
			if not rest:
				moveToHome()
				rospy.sleep(1.5)
				armrest()
				rospy.sleep(2)
			os.system('rosnode kill /open_manipulator')
			rospy.sleep(1)
			oculusprimesocket.sendString("malgcommand Q")
			oculusprimesocket.sendString("messageclients openmanipulator off")  
			oculusprimesocket.sendString("state delete arm")  
			rospy.signal_shutdown("exit")
			# TODO: kill open_manipulator_controller roslaunch 
		
		elif re.search("arm enable", s): # enable
			print("enable")
			rospy.wait_for_service('open_manipulator/set_actuator_state')
			set_actuator_state = rospy.ServiceProxy('open_manipulator/set_actuator_state', SetActuatorState)
			set_actuator_state(True)
			
		elif re.search("arm disable", s): # disable
			print("disable")
			rospy.wait_for_service('open_manipulator/set_actuator_state')
			set_actuator_state = rospy.ServiceProxy('open_manipulator/set_actuator_state', SetActuatorState)
			if not rest:
				moveToHome()
				rospy.sleep(1.5)
				armrest()
				rospy.sleep(2)
			set_actuator_state(False)
		
		elif re.search("arm open", s): # open TODO: BROKE
			print("open")
			gripperopen()
			
		elif re.search("arm close", s): # open TODO: BROKE
			print("close")
			gripperclose()
			
		elif re.search("arm home", s): # home
			print("home")
			moveToHome()
			
		elif re.search("arm rest", s): # rest
			print("rest")
			armrest()
		
		elif re.search("arm floor", s): # rest
			print("floor")
			moveToFloor()
		
		elif re.search("arm forward", s): # forward
			print("forward")
			armForward()
		
		elif re.search("arm record", s): # record1
			print("record"+s[-1])
			num = int(s[-1])-1
			recordedJointPositions[num] = getJointPosition()
			printJointPos(recordedJointPositions[num])
			
		elif re.search("arm goto", s): # rest
			print("goto"+s[-1])
			num = int(s[-1])-1
			if not recordedJointPositions[num] == None:
				rospy.wait_for_service('open_manipulator/goal_joint_space_path')
				set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
				newjointpositions = getJointPosition()
				newjointpositions.position[0] = recordedJointPositions[num].position[0]
				newjointpositions.position[1] = recordedJointPositions[num].position[1]
				newjointpositions.position[2] = recordedJointPositions[num].position[2]
				newjointpositions.position[3] = recordedJointPositions[num].position[3]
				set_joint_position('', newjointpositions , 2)
				rest = False
			
		elif re.search("arm pickup", s):
			print("pickup")
			gripperopen()
			moveToHome()
			rospy.sleep(1.5)
			moveToFloor()
			rospy.sleep(2)
			gripperclose()
			rospy.sleep(2)
			moveToHome()
			
		elif re.search("arm grab", s):
			print("grab")
			gripperopen()
			moveToHome()
			rospy.sleep(1)
			armForward()
			rospy.sleep(2)
			gripperclose()
			rospy.sleep(2.5)
			moveToHome()
			rospy.sleep(1)
			gripperopen()
			rospy.sleep(3)
			armrest()
			gripperclose()
		
		rospy.sleep(0.01) 


if __name__ == '__main__':
	main()

