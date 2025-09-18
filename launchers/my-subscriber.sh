#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
rosrun followlane my_subscriber_node.py

# wait for app to end
dt-launchfile-join