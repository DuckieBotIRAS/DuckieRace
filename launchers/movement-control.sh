#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun odometry odometry_node.py &
rosrun odometry control_point_node.py

# wait for app to end
dt-launchfile-join