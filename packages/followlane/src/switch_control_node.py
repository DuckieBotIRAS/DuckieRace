#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, UInt8
from enum import Enum

import os
from duckietown.dtros import DTROS, NodeType
from my_msg.msg import Intersection

class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Intersection = 3
    Parking = 4

class SwitchControlNode(DTROS):
    def __init__(self,node_name):
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Float64, self.cbDuckieDetected, queue_size = 1)
        #self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbLaneDetected, queue_size = 1)
        self.sub_intersection = rospy.Subscriber(f'/{self._vehicle_name}/detect/Intersection', Intersection, self.cbIntersectionDetected, queue_size = 1)
        self.sub_intersection_finished = rospy.Subscriber(f'/{self._vehicle_name}/drive/Intersection/finished', UInt8, self.cbIntersectionFinished, queue_size = 1)
        
        self.sub_parking = rospy.Subscriber(f'/{self._vehicle_name}/detect/parking', UInt8, self.cbParkingDetected, queue_size = 1)
        self.sub_paring_finished = rospy.Subscriber(f'/{self._vehicle_name}/drive/parking/finished', UInt8, self.cbParkingFinished, queue_size = 1)
        

        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size = 1)
        
        self._control_mode = ControlType.Lane

    def cbParkingDetected(self, msg):
        if msg.data == 1:
            print(f'now parking mode')
            self._control_mode = ControlType.Parking
    
    def cbParkingFinished(self, msg):
        if self._control_mode == ControlType.Parking:
            print(f'parking mode finished')
            self._control_mode = ControlType.Lane

    def cbDuckieDetected(self, msg):
        print('received message')
        # Write your own code her

    #def cbLaneDetected(self, msg):
        #print('received message')
        # Write your own code her

    def cbIntersectionDetected(self,msg):
        print(f'received intersection message {msg}')
        if msg.intersection == True:
            self._control_mode = ControlType.Intersection

    def cbIntersectionFinished(self,msg):
        print(f'received intersection message {msg}')

        if self._control_mode == ControlType.Intersection:
            self._control_mode = ControlType.Lane

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            msg_control = Int32()
            msg_control.data = self._control_mode.value
            self.pub_control.publish(msg_control)

if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    # keep the process from terminating
    rospy.spin()