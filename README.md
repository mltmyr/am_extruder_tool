# Custom AM plastic extruder.

### Getting started
`cd [your_ros2_workspace]/src`

`git clone git@github.com:mltmyr/am_extruder_tool.git`
`git clone git@github.com:mltmyr/am_extruder_msg.git`

```
(`git clone git@github.com:mltmyr/am_extruder_simple_ui.git`)

### Dependencies

#### RS-232
`cd am_extruder_tool/`

`git submodule init`

`git submodule update`

#### ros2_control
`apt install ros-foxy-ros2-control`

`apt install ros-foxy-ros2-controllers`

### Build
`source /opt/ros/foxy/setup.bash`

`cd [your_ros2_workspace]`

`colcon build`

`source install/setup.bash`


### Start extruder demo
`ros2 launch am_extruder_tool_example.launch.py`


### Listen for joint states
`ros2 topic echo /joint_states`


### Send commands to the controllers
`ros2 topic pub /filament_heater_controller/commands std_msgs/msg/Float64MultiArray 'data: {1.0}'`

`ros2 topic pub /filament_mover_controller/commands std_msgs/Float64MultiArray 'data: {1.0}'`

### (Optionally) Use rqt gui plugin
Open rqt gui with `rqt` and choose plugin am_extruder/Simple extruder GUI. If the plugin cannot be found in the list, try `rqt --force-discover` instead.
