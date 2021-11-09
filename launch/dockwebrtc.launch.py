from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	

	webrtcserver = LaunchConfiguration('webrtcserver', default='')
	peerid = LaunchConfiguration('peerid', default='')
	turnserverport = LaunchConfiguration('turnserverport', default='')
	turnserverlogin = LaunchConfiguration('turnserverlogin', default='')

	dockdevice = LaunchConfiguration('dockdevice', default='/dev/video3')

	dockoffset = LaunchConfiguration('dockoffset', default=0.018 )

	return LaunchDescription([

		Node(
			package='autocrawler',
			executable='webrtcrs',
			name='webrtcrs',
			output='screen',
			emulate_tty=True, # (for python print output to screen)
			arguments = [peerid, webrtcserver, turnserverport, 
				turnserverlogin, "--video-width=640", "--video-height=480", 
				"--video-bitrate=256", 
				],
			remappings=[ ('/image_raw', '/fiducial_images'), ]
		),

		Node(
			package='autocrawler',
			executable='webrtc_status_listener.py',
			name='webrtc_status_listener',
			output='screen',
			emulate_tty=True,
		),

		# ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video6		
		# /camera_info
		# /image_raw
		# /parameter_events
		# /rosout
		Node(
			package='usb_cam',
			executable='usb_cam_node_exe',
			name='usb_cam_node_exe',
			output='screen', # TODO: nuke
			parameters=[{'video_device':dockdevice, "frame_id":"usb_cam",
				"camera_info_url": "package://autocrawler/config/dockcam_ost.yaml",
				"camera_name": "rear_camera",
				}],
		),
	
		Node(
			package='autocrawler',
			executable='docktargettostate.py',
			name='docktargettostate',
			output='screen',
			emulate_tty=True,
			parameters=[{"dock_offset": dockoffset}],
		),
	
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name='base_to_dockcam_broadcaster',
			output='screen',
			arguments=['-0.157', '-0.003', '0.080', '3.14159', '0', '0', "base_link", "dockcam"]
		),


		# topics produced by above
		# /camera_info
		# /image_raw
		# /parameter_events
		# /rosout
		# /tf_static
		# /webrtcstatus

		# subscribes: /camera/image, /camera/camera_info

		Node(
			package='aruco_detect',
			namespace='aruco_detect',
			executable='aruco_detect',
			name='aruco_detect',
			parameters=[{
				"publish_images": True,
				"fiducial_len": 0.065,
				"dictionary": 1,
				"do_pose_estimation": True,
				"publish_fiducial_tf": True,
				"ignore_fiducials": "0-16,18-999",
			}],
			remappings=[
				('/camera/image', '/image_raw'),
				('camera/camera_info', '/camera_info'), 
			]
		)

	])
    
