#!/usr/bin/env python3

from enum import Enum
import rospy
import tf   
import numpy as np
from std_msgs.msg import Float64,UInt8
from geometry_msgs.msg import Point
from enum import Enum
from nav_msgs.msg import Odometry
#from followlane.msg import MyOdom
from std_msgs.msg import Float64
import math


from my_msg.msg import OdomPosition
from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
class LaneDetected(Enum):
    BOTH_LINES = 0
    WHITE_LINE = 1
    YELLOW_LINE = 2
    NO_LINE = 3

class ControlMoveNode(DTROS):
    def __init__(self,node_name):
        super(ControlMoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        

        vehicle_name = os.environ['VEHICLE_NAME']
        
        point_topic = f"/{vehicle_name}/detect/point"
        self.pub_point = rospy.Publisher(point_topic, Point, queue_size = 1)

        forward_topic = f"/{vehicle_name}/drive/forward"
        self.pub_forward = rospy.Publisher(forward_topic, Float64, queue_size = 1)

        turn_topic = f"/{vehicle_name}/drive/turn"
        self.pub_turn = rospy.Publisher(turn_topic, Float64, queue_size = 1)



    def driveToPoint(self, x,y):
        print(f'received message ({x}, {y})')
        msg = Point(x=x,y=y,z=0)
        self.pub_point.publish(msg)

    def forward(self, value):        
        print(f'forward : {value}')
        msg = Float64()
        msg.data = value
        self.pub_forward.publish(msg)

    def turn(self, value):
        print(f'turn : {value}')
        msg = Float64()
        msg.data = value
        self.pub_turn.publish(msg)

    def cbFinishedMove(self, msg):
        print('arrived')

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            action = input("action \n")

            if action == 'f':
                value = float(input('value \n'))
                self.forward(value)
            elif action == 't':
                value = float(input('value \n'))
                self.turn(value)
            elif action == 'p':
                x = float(input('x \n'))
                y = float(input('y \n'))
                self.driveToPoint(x,y)

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = ControlMoveNode(node_name='control_move_node')
    node.run()
    #node.cbDriveToPoint(Point(x=1,y=1,z=0))
    # keep the process from terminating
    rospy.spin()