

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description	():

	colorwidth = LaunchConfiguration('color_width', default='640')
	colorheight = LaunchConfiguration('color_height', default='480')
	colorfps = LaunchConfiguration('color_fps', default='15')
	enabledepth = LaunchConfiguration('enable_depth', default='true')
	initialreset = LaunchConfiguration('initial_reset', default='false') 

	return launch.LaunchDescription([

		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name='base_to_realsense_broadcaster',
			output='screen',
			arguments=['0.152', '0.012', '0.163', '0', '0', '0', "base_link", "camera_link"]
		),

		Node(
			package='realsense2_camera',
			executable='realsense2_camera_node',
			name='camera',
			output='screen',
			parameters = [{'color_width': colorwidth,
				'color_height': colorheight,
				'color_fps': colorfps,
				'enable_depth': enabledepth,
				'initial_reset': initialreset,
			}]
		),
	])


