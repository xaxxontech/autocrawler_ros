import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
			package='tf2_ros',
			executable='static_transform_publisher',
			name='base_to_lidar_broadcaster',
			output='screen',
			arguments=['0.103', '0.0', '0.120', '0.0', '0.0', '0.0', "base_link", "laser_frame"]
		),
		
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name='link_to_footprint_broadcaster',
			output='screen',
			arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', "base_link", "base_footprint"]
		),
		
		IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 
				'launch'), '/bringup_launch.py']),
            launch_arguments={
                'map': os.path.join(
					get_package_share_directory('autocrawler'), 'maps', 'map.yaml'),
                'use_sim_time': 'false',
                'params_file': os.path.join(
					get_package_share_directory('autocrawler'),
					'config', 'nav_params.yaml')
			}.items(),
        ),

	])
