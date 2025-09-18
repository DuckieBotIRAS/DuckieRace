#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
#rosrun followlane camera_tkinter_node.py &
rosrun followlane detect_lane_node.py &
rosrun followlane control_lane_node.py & 
#rosrun followlane detect_intersection_node.py &
rosrun followlane control_parking_node.py &
rosrun odometry control_point_node.py &
rosrun odometry odometry_node.py &
rosrun followlane switch_control_node.py
# wait for app to end
dt-launchfile-join