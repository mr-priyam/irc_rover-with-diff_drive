# IRC Rover with Differential_Drive mechanism for ROS2 HUMBLE




## Requirements
- ROS 2 Humble
- colcon build
- meshes referenced in `irc_rover/meshes` (already in workspace)

## Quick build & run 
1. From workspace root :
```sh
colcon build
```
2. Source the install setup:
```sh
source install/setup.bash
```

3.launch the XML variant:
```sh
ros2 launch irc_rover display.launch.xml
```

3.Make sure to choose osom as a Fixed Frame . Now in another terminal after sourcing write:
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```

Now you can control the rover using teleop_twist.
