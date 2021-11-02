from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description	():

	webrtcserver = LaunchConfiguration('webrtcserver', default='')
	peerid = LaunchConfiguration('peerid', default='')
	audiodevice = LaunchConfiguration('audiodevice', default='')
	videowidth = LaunchConfiguration('videowidth', default='')
	videoheight = LaunchConfiguration('videoheight', default='')
	videobitrate = LaunchConfiguration('videobitrate', default='')
	turnserverport = LaunchConfiguration('turnserverport', default='')
	turnserverlogin = LaunchConfiguration('turnserverlogin', default='')
	
	return LaunchDescription([

		Node(
			package='autocrawler',
			executable='webrtcrs',
			name='webrtcrs',
			output='screen',
			emulate_tty=True, # (for python print output to screen)
			arguments = [peerid, webrtcserver, audiodevice, videowidth, videoheight,
				videobitrate, turnserverport, turnserverlogin],
			# remappings=[
				# ('/image_raw', '/camera/color/image_raw'),
			# ]
		),

		Node(
			package='autocrawler',
			executable='webrtc_status_listener.py',
			name='webrtc_status_listener',
			output='screen',
			emulate_tty=True,
		)
	])

