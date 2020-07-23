from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description	():
	
	colorwidth = LaunchConfiguration('color_width', default='640')
	colorheight = LaunchConfiguration('color_height', default='480')
	colorfps = LaunchConfiguration('color_fps', default='15')
	enabledepth = LaunchConfiguration('enable_depth', default='true')
	initialreset = LaunchConfiguration('initial_reset', default='false') # dummy for ROS1 compatibility

	return LaunchDescription([

		Node(
			package='tf2_ros',
			node_executable='static_transform_publisher',
			node_name='base_to_realsense_broadcaster',
			output='screen',
			arguments=['0.152', '0.012', '0.163', '0', '0', '0', "base_link", "camera_link"]
        ),

		Node(
			package='realsense_node',
			node_executable='realsense_node',
			node_name='rgbd_node',
			output='screen',
			parameters=[{'color0.resolution' : [[colorwidth], [colorheight]] },
				{'color0.fps' : colorfps},
				{'depth0.enabled' : enabledepth },
			],
        )
    ])



"""

ros2 launch autocrawler rgbwebrtc.launch.py peerid:=--peer-id=505283304 webrtcserver:=--server=wss://xaxxon.com:8443
videowidth:=--video-width=1280 videoheight:=--video-height=720 videobitrate:=--video-bitrate=512 turnserverport:=--turnserver-port=3478
turnserverlogin:=--turnserver-login=auto:robot
<launch>
    
	<!-- realsense depth and colour -->
	<node pkg="tf2_ros" type="static_transform_publisher" name="base_to_realsense_broadcaster" 
		args="0.152 0.012 0.163 0 0 0 base_link camera_link" />
    
	<arg name="camera"                default="camera"/>
	<arg name="color_width" default="640"/>
	<arg name="color_height" default="480"/>
	<arg name="color_fps" default="15"/>
	<arg name="enable_depth" default="false"/>
	<arg name="initial_reset" default="false"/>
	<include file="$(find realsense2_camera)/launch/rs_camera.launch">
		<arg name="enable_infra1"     value="false"/>
		<arg name="enable_infra2"     value="false"/>
		<arg name="enable_fisheye"    value="false"/>
		<arg name="enable_gyro"       value="false"/>
		<arg name="enable_accel"      value="false"/>

		<arg name="enable_depth"      value="$(arg enable_depth)"/>
		<arg name="depth_width"       value="480"/> 
		<arg name="depth_height"      value="270"/> 
		<arg name="depth_fps"         value="30"/>
		<arg name="clip_distance"	value="2.5"/>
		<arg name="enable_color"      value="true"/>
		<arg name="color_width"       value="$(arg color_width)"/>
		<arg name="color_height"      value="$(arg color_height)"/>
		<arg name="color_fps"      value="$(arg color_fps)"/>

		<arg name="initial_reset" value="$(arg initial_reset)"/>
    </include>

</launch>

$ ros2 param list
/camera:
  align_depth
  base_frame_id
  color0.enabled
  color0.fps
  color0.resolution
  dense_pointcloud
  depth0.enabled
  depth0.fps
  depth0.resolution
  enable_pointcloud
  infra1.enabled
  infra1.fps
  infra1.resolution
  infra2.enabled
  infra2.fps
  infra2.resolution
  serial_no
  use_sim_time

camera:
  ros__parameters:
    align_depth: true
    base_frame_id: camera_link
    color0:
      enabled: true
      fps: 30
      resolution: !!python/object/apply:array.array
      - q
      - - 640
        - 480
    dense_pointcloud: true
    depth0:
      enabled: true
      fps: 30
      resolution: !!python/object/apply:array.array
      - q
      - - 640
        - 480
    enable_pointcloud: true
    infra1:
      enabled: true
      fps: 30
      resolution: !!python/object/apply:array.array
      - q
      - - 640
        - 480
    infra2:
      enabled: true
      fps: 30
      resolution: !!python/object/apply:array.array
      - q
      - - 640
        - 480
    serial_no: null
    use_sim_time: false
"""
