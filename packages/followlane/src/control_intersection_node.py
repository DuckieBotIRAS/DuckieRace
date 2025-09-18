#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, String, UInt8

from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType
import yaml
from my_msg.msg import Intersection
import random

class ControlIntersetionNode(DTROS):
    def __init__(self,node_name):
        super(ControlIntersetionNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.enable = False
        self.wait_finished_movement = False
        self.is_running = False

        self._vehicle_name = os.environ['VEHICLE_NAME']
        intersection_topic = f'/{self._vehicle_name}/detect/Intersection'
        intersection_finished_topic = f'/{self._vehicle_name}/drive/Intersection/finished'

        self.pub_finished = rospy.Publisher(intersection_finished_topic, UInt8, queue_size = 1)
        self.sub_intersection = rospy.Subscriber(intersection_topic, Intersection, self.cbIntersection, queue_size = 1)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl , queue_size = 1)

        
        forward_topic = f"/{self._vehicle_name}/drive/forward"
        self.pub_forward = rospy.Publisher(forward_topic, Float64, queue_size = 1)

        turn_topic = f"/{self._vehicle_name}/drive/turn"
        self.pub_turn = rospy.Publisher(turn_topic, Float64, queue_size = 1)

        move_finished_topic = f"/{self._vehicle_name}/drive/finished"
        self.sub_finished_movement = rospy.Subscriber(move_finished_topic, UInt8, self.cbFinishedMove, queue_size = 1)


        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self.sub_config = rospy.Subscriber(f"/{self._vehicle_name}/conf", String, self.cbUpdateConf, queue_size = 1)

        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

    def cbFinishedMove(self,msg):
        self.wait_finished_movement = False

    def cbControl(self,msg):
        if msg.data == ControlType.Intersection.value:
            self.enable = True
        else:
            self.enable = False

    def cbIntersection(self, msg):
        print(f'received message. enabled : {self.enable}')

        if not self.enable:
            return       

        if self.is_running:
            return

        self.is_running = True 
        
        self.stop()
        rospy.sleep(2)
        possible_turns = []

        if msg.right:
            possible_turns.append('right')

        if msg.left:
            possible_turns.append('left')
        
        if msg.straight:
            possible_turns.append('straight')

        self.turn(possible_turns)

    def turn(self,possible_turns):
        if len(possible_turns) < 1:
            self.is_running = False
            return
        
        rate = rospy.Rate(1)
        turn_direction = random.choice(possible_turns)

        if turn_direction == 'right':
            turn = -80
            forward = 0.4

        if turn_direction == 'left':
            turn = 80
            forward = 0.8

        if turn_direction == 'straight':
            turn = 0
            forward = 1


        self.pub_forward.publish(Float64(forward))

        self.wait_finished_movement = True
        while self.wait_finished_movement:
            rate.sleep()

        self.pub_turn.publish(Float64(turn))
        
        self.wait_finished_movement = True
        while self.wait_finished_movement:
            rate.sleep()
        
        if turn_direction == 'left':
            self.pub_forward.publish(Float64(0.6))

            self.wait_finished_movement = True
            while self.wait_finished_movement:
                rate.sleep()

        self.pub_finished.publish(UInt8(1))
        rospy.sleep(1)
        self.is_running = False
                

    def stop(self):
        rospy.loginfo("stoping ducki. cmd_vel will be 0")

        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

    def load_conf(self,path):
        with open(path,'r') as f:
            conf_yml = f.read()
        self.update_conf(conf_yml)

    def cbUpdateConf(self,conf_msg):
        self.update_conf(conf_msg.data)
    
    def update_conf(self,conf_yml):
        self.conf = yaml.safe_load(conf_yml)
        #TODO

if __name__ == '__main__':
    # create the node
    node = ControlIntersetionNode(node_name='control_intersection_node')
    # keep the process from terminating
    rospy.spin()