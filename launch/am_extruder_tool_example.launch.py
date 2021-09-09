import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
	description_file_params = {'serial_port': 'ttyS5', 'baud_rate': '9600'}

	extruder_description_path = os.path.join(
		get_package_share_directory('am_extruder_tool'), 'description', 'am_extruder_tool_example.urdf.xacro')
	extruder_description_config = xacro.process_file(extruder_description_path, mappings=description_file_params)
	extruder_description = {'robot_description': extruder_description_config.toxml()}

	extruder_controller = os.path.join(
		get_package_share_directory('am_extruder_tool'), 'config', 'extruder_controllers.yaml')

	controller_manager_node = Node(
		package='controller_manager',
		executable='ros2_control_node',
		parameters=[extruder_description, extruder_controller],
		output={
			'stdout': 'screen',
			'stderr': 'screen',
		},
	)

	extruder_communication_helper_node = Node(
		package='am_extruder_tool',
		executable='am_extruder_com',
		output='both',
		parameters=[description_file_params],
	)

	extruder_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		parameters=[extruder_description],
		output='both',
		
	)

	heater_controller_spawner_node = Node(
		package='controller_manager',
		executable='spawner.py',
		parameters=[extruder_controller],
		arguments=['filament_heater_controller'],
		output='both',
	)

	mover_controller_spawner_node = Node(
		package='controller_manager',
		executable='spawner.py',
		parameters=[extruder_controller],
		arguments=['filament_mover_controller'],
		output='both',
	)

	joint_state_controller_spawner_node = Node(
		package='controller_manager',
		executable='spawner.py',
		parameters=[extruder_controller],
		arguments=['joint_state_controller'],
		output='both',
	)

	return LaunchDescription([
		controller_manager_node,
		extruder_communication_helper_node,
		extruder_state_publisher_node,
		
		heater_controller_spawner_node,
		mover_controller_spawner_node,
		joint_state_controller_spawner_node,
	])
