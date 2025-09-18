#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, String

from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType
import yaml

class ControlLaneNode(DTROS):
    def __init__(self,node_name):
        super(ControlLaneNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.enable = False
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        self.sub_lane = rospy.Subscriber(f'/{self._vehicle_name}/detect/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl , queue_size = 1)
        self.lastError = 0

        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self.sub_config = rospy.Subscriber(f"/{self._vehicle_name}/conf", String, self.cbUpdateConf, queue_size = 1)
        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self,msg):
        if msg.data == ControlType.Lane.value:
            self.enable = True
        
        else:
            self.enable = False

    def cbFollowLane(self, desired_center):

        print(f'ControlLane received message. enabled : {self.enable}')

        if not self.enable:
            return        
        
        center = desired_center.data
        self.followLane(center)

    def followLane_not_working(self, center):
        # Write your code for a PID controller here
        error = (center - 500) / 100

        v = 0.2
        a = 0.01 * error
        
        twist = Twist2DStamped(v=v, omega=a)
        print(f'moving {v} {a} error {error}')
        self.pub_cmd_vel.publish(twist)

    # error between 1 and -1
    def followLane(self, error):
        #error = -error
        #error = center - 50

        #Kp = 0.0025
        #Kd = 0.007

        Kp = self.kp / 10 #0.0125 * 2
        Kd = self.ki / 10 #0.035 * 2

        print(f'error {error}')

        a = -(Kp * error + Kd * (error - self.lastError) )

        #if a < -0.99:
        #    a = -0.99

        #if a > 0.99:
        #    a = 0.99

        self.lastError = error
        
        # twist.linear.x = 0.05        
        v = min(self.MAX_VEL * ((1 - abs(error)) ** 2), 0.5)
        v = max(v,0.1)
        twist = Twist2DStamped(v=v, omega=a)
        print(f'moving {v} {a} error {error}')
        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

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
        self.MAX_VEL = self.conf['pid_regler']['max_vel'] / 100
        self.kp = self.conf['pid_regler']['p']
        self.kd = self.conf['pid_regler']['d']
        self.ki = self.conf['pid_regler']['i']

if __name__ == '__main__':
    # create the node
    node = ControlLaneNode(node_name='control_lane_node')
    # keep the process from terminating
    rospy.spin()