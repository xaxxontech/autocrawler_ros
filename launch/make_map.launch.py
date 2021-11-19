import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription

def generate_launch_description():

	return LaunchDescription([
		Node(
			package='xaxxon_openlidar',
			executable='lidarbroadcast',
			name='lidarbroadcast',
			parameters=[os.path.join(get_package_share_directory('autocrawler'), 'config',
				'lidar_params.yaml')],
			output='screen',
			emulate_tty=True,
		),
		
		Node(
			package='autocrawler',
			executable='odom_tf.py',
			name='odom_tf',
			output='screen',
			emulate_tty=True,
		),
		
		Node(
			package='autocrawler',
			executable='map_remote.py',
			output='screen',
			emulate_tty=True,
		),
		
		# add rotated laser frame broadcaster etc
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name='base_to_lidar_broadcaster',
			output='screen',
			arguments=['0.103', '0.0', '0.120', '0.0', '0.0', '0.0', "base_link", "laser_frame"]
		),
		
		Node(
			package='cartographer_ros',
			executable='cartographer_node',
			output='screen',
			parameters=[{'use_sim_time': False}],
			arguments=['-configuration_directory', os.path.join(get_package_share_directory('autocrawler'), 'config'),
				'-configuration_basename', 'cartographer.lua'],
		),
		
		Node(
			package='cartographer_ros',
			executable='occupancy_grid_node',
			output='screen',
			parameters=[{'use_sim_time': False}],
			arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
		),

	])

