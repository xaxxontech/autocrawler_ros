#!/usr/bin/env python3

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
positionsfromfile = []
startupok = False
rest = True

sep = os.path.sep
settingsfile = os.environ.get("CATALINA_HOME")+sep+"conf"+sep+"openmanipulator.txt"

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
	
	pos = getfileposition("floor")
	if not pos: 
		pos = [-0.0322135984898, 1.44500994682, -0.655009806156, 0.786932170391]

	newjointpositions = getJointPosition()
	newjointpositions.position[0] = pos[0]
	newjointpositions.position[1] = pos[1]
	newjointpositions.position[2] = pos[2]
	newjointpositions.position[3] = pos[3]
	set_joint_position('', newjointpositions , 2)	
	rest=False
	
def moveToHome():
	global rest
	rospy.wait_for_service('open_manipulator/goal_joint_space_path')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
	
	pos = getfileposition("home")
	if not pos: 
		pos = [-0.00153398083057, -1.05231082439, 0.369689375162, 0.701029241085]

	newjointpositions = getJointPosition()
	newjointpositions.position[0] = pos[0]
	newjointpositions.position[1] = pos[1]
	newjointpositions.position[2] = pos[2]
	newjointpositions.position[3] = pos[3]
	
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
	
	pos = getfileposition("rest")
	if not pos: 
		pos = [-0.00306796166115, -1.56466042995, 1.45728182793, 0.506213665009]

	newjointpositions = getJointPosition()
	newjointpositions.position[0] = pos[0]
	newjointpositions.position[1] = pos[1]
	newjointpositions.position[2] = pos[2]
	newjointpositions.position[3] = pos[3]
	
	set_joint_position('', newjointpositions , 2)
	rest = True

def armForward():
	global rest
	rospy.wait_for_service('open_manipulator/goal_joint_space_path')
	set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
	
	pos = getfileposition("forward")
	if not pos: 
		pos = [-0.00153398083057, 0.00766990426928, 0.0291456356645, 0.00613592332229]

	newjointpositions = getJointPosition()
	newjointpositions.position[0] = pos[0]
	newjointpositions.position[1] = pos[1]
	newjointpositions.position[2] = pos[2]
	newjointpositions.position[3] = pos[3]

	set_joint_position('', newjointpositions , 2)
	rest = False


def loadpositionsfromfile():
	global positionsfromfile
	
	f = open(settingsfile)
	Lines = f.readlines()
	f.close()

	for line in Lines:

		values = line.split() 
		if not values: 		  # blank line	
			continue

		if line.strip()[0] == "#": # skip comments
			continue

		positionsfromfile.append(values)
				
	print(positionsfromfile)


def savepositionstofile():
	
	if not positionsfromfile:
		return
		
	positions = positionsfromfile[:]

	f = open(settingsfile)
	Lines = f.readlines()
	f.close()
	
	f = open(settingsfile, "w")
	
	for line in Lines:

		values = line.split() 
		if not values: 		  # blank line	
			f.write(line)
			continue

		if line.strip()[0] == "#": # comment
			f.write(line)
			continue

		i = 0
		for pos in positions:
			
			if values[0] == pos[0]:
				line = ""
				for s in pos:
					line += s + " "
				line = line.strip()+"\n"
				f.write(line)
				del positions[i]
				break
				
			i += 1	
	
	# write any new positions leftover to the end
	for pos in positions:
		line = ""
		for s in pos:
			line += s + " "
		line = line.strip()+"\n"
		f.write(line)
		
	f.close()
	print(settingsfile+" saved")
	
	
def getfileposition(name):

	if not positionsfromfile:
		return None

	for filepos in positionsfromfile:
		if filepos[0] == name:
			return [float(filepos[1]), float(filepos[2]), float(filepos[3]), float(filepos[4])]
			
	return None
	
def recordjointposition(num):
	global positionsfromfile
	#  recordedJointPositions[num] = getJointPosition()
	#  printJointPos(recordedJointPositions[num])
	
	name = "user"+str(num)
	p = getJointPosition()
	values = [name, str(p.position[0]), str(p.position[1]), str(p.position[2]), str(p.position[3])]
	
	# check if already in list
	i = 0
	for filepos in positionsfromfile:
		if filepos[0] == name:
			positionsfromfile[i] = values
			values = None
		i += 1
		
	if values:
		positionsfromfile.append(values)
	
	savepositionstofile()
			
	
	
def main(args=None):
	global rest
	
	rospy.init_node('autocrawler_openmanipulator', anonymous=False)
	rospy.Subscriber("open_manipulator/joint_states", JointState, jointCallback) 
	
	oculusprimesocket.connect()	
	oculusprimesocket.sendString("log autocrawler_openmanipulator.py connected")  
	oculusprimesocket.sendString("state delete arm")  

	loadpositionsfromfile()

	while not rospy.is_shutdown():
		s = oculusprimesocket.replyBufferSearch("<state> arm")
		
		if re.search("arm on", s): # enable
			oculusprimesocket.sendString("malgcommand W")
			# TODO: send open_manipulator_controller roslaunch command 
			
		elif re.search("arm off", s): # arm power off macro
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
			# TODO: kill open_manipulator_controller roslaunch, currently handled by javascript
		
		elif re.search("arm enable", s): # enable servos
			print("enable")
			rospy.wait_for_service('open_manipulator/set_actuator_state')
			set_actuator_state = rospy.ServiceProxy('open_manipulator/set_actuator_state', SetActuatorState)
			set_actuator_state(True)
			oculusprimesocket.sendString("messageclients servos enabled")  
			
		elif re.search("arm disable", s): # disable servos macro 
			print("disable")
			rospy.wait_for_service('open_manipulator/set_actuator_state')
			set_actuator_state = rospy.ServiceProxy('open_manipulator/set_actuator_state', SetActuatorState)
			#  if not rest:
				#  moveToHome()
				#  rospy.sleep(1.5)
				#  armrest()
				#  rospy.sleep(2)
			set_actuator_state(False)
			oculusprimesocket.sendString("messageclients servos disabled")  
		
		elif re.search("arm open", s): # gripper open 
			print("open")
			gripperopen()
			
		elif re.search("arm close", s): # gripper close
			print("close")
			gripperclose()
			
		elif re.search("arm home", s): # home
			print("home")
			moveToHome()
			
		elif re.search("arm rest", s): # rest
			print("rest")
			armrest()
		
		elif re.search("arm floor", s): # floor
			print("floor")
			moveToFloor()
		
		elif re.search("arm forward", s): # forward
			print("forward")
			armForward()
		
		elif re.search("arm record", s): # record pos # 
			print("record "+s[-1])
			num = int(s[-1])-1
			recordjointposition(num)
			oculusprimesocket.sendString("messageclients joint #"+str(num+1)+" saved")  
			
		elif re.search("arm goto", s): # user pos #
			print("goto"+s[-1])
			num = int(s[-1])-1
			
			pos = getfileposition("user"+str(num))
			if not pos == None:
				rospy.wait_for_service('open_manipulator/goal_joint_space_path')
				set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
				newjointpositions = getJointPosition()
				newjointpositions.position[0] = pos[0]
				newjointpositions.position[1] = pos[1]
				newjointpositions.position[2] = pos[2]
				newjointpositions.position[3] = pos[3]
				set_joint_position('', newjointpositions , 2)
				rest = False
				
			#  if not recordedJointPositions[num] == None:
				#  rospy.wait_for_service('open_manipulator/goal_joint_space_path')
				#  set_joint_position = rospy.ServiceProxy('open_manipulator/goal_joint_space_path', SetJointPosition)
				#  newjointpositions = getJointPosition()
				#  newjointpositions.position[0] = recordedJointPositions[num].position[0]
				#  newjointpositions.position[1] = recordedJointPositions[num].position[1]
				#  newjointpositions.position[2] = recordedJointPositions[num].position[2]
				#  newjointpositions.position[3] = recordedJointPositions[num].position[3]
				#  set_joint_position('', newjointpositions , 2)
				#  rest = False
			
		elif re.search("arm pickup", s): # macro
			print("pickup")
			gripperopen()
			moveToHome()
			rospy.sleep(1.5)
			moveToFloor()
			rospy.sleep(2)
			gripperclose()
			rospy.sleep(2)
			moveToHome()
			
		elif re.search("arm grab", s): # macro
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

