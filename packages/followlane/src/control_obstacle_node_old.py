#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from enum import Enum

from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType

class ControlObstacleNode(DTROS):
    def __init__(self,node_name):
        super(ControlObstacleNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.enable = False
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Float64, self.cbAvoideObstacle, queue_size = 1)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl, queue_size = 1)
        
        
        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self,msg):
        if msg.data == ControlType.Obstacle.value:
            self.enable = True
        
        else:
            self.enable = False

    def cbAvoideObstacle(self, msg):
        print('received message')

        if not self.enable:
            return
        
        # Write your code for Obstacle Avoidance here

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

if __name__ == '__main__':
    # create the node
    node = ControlObstacleNode(node_name='control_obstacle_node')
    # keep the process from terminating
    rospy.spin()