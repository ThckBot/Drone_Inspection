#!/bin/bash

sudo chmod 666 /dev/ttyTHS1

# Launch roscore in its own terminal
# gnome-terminal -- roscore
gnome-terminal --command="bash -c 'cd ~/Drone_Inspection/catkin_ws; roscore; $SHELL'"

# Launch the px4
sleep 5
# gnome-terminal -- roslaunch mavros px4.launch
gnome-terminal --command="bash -c 'cd ~/thirdparty/vio_ws/devel/; source setup.bash; roslaunch px4_realsense_bridge bridge_mavros.launch; $SHELL'"

# Publish odometry pose at 100 Hz note odometry pose is 331
# sleep 2
# gnome-terminal -- rosservice call /mavros/set_message_interval 331 100
# gnome-terminal --command="bash -c 'cd ~/Drone_Inspection/catkin_ws; rosservice call /mavros/set_message_interval 331 100 $SHELL'"

# Check the odometry in is working
# sleep 2
# rostopic hz /mavros/odometry/in

