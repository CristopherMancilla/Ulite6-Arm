# Ulite6_arm with blockchain Project

## Directory path of project
$ ~/ros2_ws/src/ulite6_arm

## Ulite6 Gazebo 

### Start simulation in Gazebo environment
$ ~/ros2_ws
$ ros2 launch ulite6_arm_gazebo ulite6_table_gazebo_launch.py

### Start trajectory test in Gazebo
- Execute a trajectory test in Gazebo, the robot draws a square of 15.0x15.0 cm and
equilateral triangle of height 15 cm.

$ ~/ros2_ws
$ ros2 launch ulite6_arm_gazebo ulite6_draw_square_launch.py

- To see which jointTrajectoryController is executing the command is

$ ros2 control list_hardware_interfaces

- joint{i}/velocity must be shows by the terminal
 



