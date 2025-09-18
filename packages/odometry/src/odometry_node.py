#!/usr/bin/env python3

from typing import Tuple
import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np 
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float64

from my_msg.msg import OdomPosition

#from followlane.msg import MyOdom
#import matplotlib.pyplot as plt
import cv2 as cv

class OdometryNode(DTROS):
    def __init__(self,node_name):
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        # temporary data storage
        self._ticks_left = 0
        self._ticks_right = 0


        self._ticks_left_prev = 0
        self._ticks_right_prev = 0
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        odom_topic = f"/{self._vehicle_name}/odom/position"
        
        self.pub_pos = rospy.Publisher(odom_topic, OdomPosition, queue_size = 1)
        #self.pub_pos_x = rospy.Publisher(odom_topic + 'X', Float64, queue_size = 1)
        #self.pub_pos_y = rospy.Publisher(odom_topic + 'Y', Float64, queue_size = 1)
        #self.pub_pos_theta = rospy.Publisher(odom_topic + 'Theta', Float64, queue_size = 1)


        self.n_tot = 150#135 # total number of ticks per revolution
        self.alpha = 2 * np.pi / self.n_tot # wheel rotation per tick in radians

        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0

        self._x_positions = []
        self._y_positions = []
        self._theta_start = 0.0

        self.r_ticks = []
        self.l_ticks = []

        self._window = 'frame'
        self._image = np.zeros([1000,1000,3])
        cv.namedWindow(self._window, cv.WINDOW_AUTOSIZE)

        self._image = cv.circle(self._image,(int(0) + 500,int(0) + 500),2,(255,0,0))

        #change_n_tot = lambda x : self.change_n(x)
        #cv.createTrackbar('n_ticks', self._window, 0, 1000, change_n_tot)
        #cv.setTrackbarPos('n_ticks',self._window,self.n_tot)
        cv.imshow(self._window, self._image)
        cv.waitKey(1)


        
        print(f"The angular resolution of our encoders is: {np.rad2deg(self.alpha)} degrees")

    #def change_n(self,x):
    #    print('executing')
    #    self.n_tot = x
    #    self.alpha = 2 * np.pi / self.n_tot
    #    self._image = np.zeros([1000,1000,3])
#
    #    ticks_left_prev = self.l_ticks[0]
    #    ticks_right_prev = self.r_ticks[0]
#
    #    self.x_prev = 0.0
    #    self.y_prev = 0.0
    #    self.theta_prev = 0.0
    #    self._x_positions = []
    #    self._y_positions = []
#
    #    print(f'alpha now : {self.alpha}')
#
    #    for i in range(len(self.r_ticks)):
    #        
    #        ticks_left = self.l_ticks[i]
    #        ticks_right = self.r_ticks[i]
    #        #x_curr = self._x_positions[i]
    #        #y_curr = self._y_positions[i]
    #        #pos = (int((x_curr - self._x_positions[0]) *250) + 500,500 - int((y_curr - self._y_positions[0])*250))
    #        #self._image = cv.circle(self._image,pos,2,(255,0,0))
#
#
    #        #print(f'updating position r:{self._ticks_right - self._ticks_right_prev } l:{self._ticks_left - self._ticks_left_prev} with r:{self._ticks_right} l:{self._ticks_left}')
    #        delta_phi_left = self.alpha * (ticks_left - ticks_left_prev)
    #        delta_phi_right = self.alpha * (ticks_right - ticks_right_prev)
    #        ticks_left_prev = ticks_left
    #        ticks_right_prev = ticks_right
    #        #self._ticks_right = 0
    #        #self._ticks_left = 0
    #        self.pose_estimation(delta_phi_left,delta_phi_right)
#
    #    cv.imshow(self._window, self._image)
    #    cv.waitKey(1)


    
    def callback_left(self, data):
        self._ticks_left = data.data

    def callback_right(self, data):
        self._ticks_right = data.data

    def run(self):
        rate = rospy.Rate(20) #TODO change back to 60?
        while not rospy.is_shutdown():
            if np.abs(self._ticks_right -self._ticks_right_prev) > 0.1 or np.abs(self._ticks_left - self._ticks_left_prev) > 0.1:
                self.r_ticks.append(self._ticks_right)
                self.l_ticks.append(self._ticks_left)

                print(f'updating position r:{self._ticks_right - self._ticks_right_prev} l:{self._ticks_left - self._ticks_left_prev} with r:{self._ticks_right} l:{self._ticks_left}')
                delta_phi_left = self.alpha * (self._ticks_left - self._ticks_left_prev)
                delta_phi_right = self.alpha * (self._ticks_right - self._ticks_right_prev)
                self._ticks_left_prev = self._ticks_left
                self._ticks_right_prev = self._ticks_right
                #self._ticks_right = 0
                #self._ticks_left = 0

                if len(self.r_ticks) == 1:
                    delta_phi_left = 0
                    delta_phi_right = 0

                self.pose_estimation(delta_phi_left,delta_phi_right)
                print(f' ticks : left {self._ticks_left} right {self._ticks_right}')
            
            if len(self._x_positions) > 0:
                msg = OdomPosition()
                msg.x = self.x_prev - self._x_positions[0]
                msg.y = self.y_prev - self._y_positions[0]
                msg.theta = ((self.theta_prev) / (2 * np.pi) * 360  +180 )% 360
                self.pub_pos.publish(msg)
                print(f'published {msg}')

            #msg_x = Float64()
            #msg_x.data = self.x_prev
            #self.pub_pos_x.publish(msg_x)

            #msg_y = Float64()
            #msg_y.data = self.y_prev
            #self.pub_pos_y.publish(msg_y)

            #msg_t = Float64()
            #msg_t.data = self.theta_prev
            #self.pub_pos_theta.publish(msg_t)

            cv.imshow(self._window, self._image)
            cv.waitKey(1)
            
            rate.sleep()

    def pose_estimation(
        self,
        #R: float,
        #baseline: float,
        #x_prev: float,
        #y_prev: float,
        #theta_prev: float,
        delta_phi_left: float,
        delta_phi_right: float,
    ) :
        baseline = 0.1
        R = 0.0318
        x_curr = self.x_prev + R * (delta_phi_left+delta_phi_right)*np.cos(self.theta_prev)/2 / 0.6
        y_curr = self.y_prev + R * (delta_phi_left+delta_phi_right)*np.sin(self.theta_prev)/2 / 0.6
        theta_curr = self.theta_prev + R*(delta_phi_right-delta_phi_left)/(baseline)

        self.x_prev = x_curr
        self.y_prev = y_curr
        self.theta_prev = theta_curr

        if len(self._x_positions) < 10:
            self._x_positions.append(x_curr)
            self._y_positions.append(y_curr)

        print(f'current position : {x_curr - self._x_positions[0]}, {y_curr - self._y_positions[0]} : {(theta_curr) / (2 * np.pi) * 360 % 360}')

        #print(f'x: \n{[self._x_positions[i*100] for i in range(int(len(self._x_positions) / 100))]} \n')
        #print(f'y: \n{[self._y_positions[i*100] for i in range(int(len(self._y_positions) / 100))]} \n')
        #fig, ax = plt.subplots()
        #ax.plot([0, 5], [0, 5])

        #fig.canvas.draw()
        #image = #np.array(fig.canvas.renderer.buffer_rgba())
        pos = (int((x_curr - self._x_positions[0]) *250) + 500,500 - int((y_curr - self._y_positions[0])*250))
        print(f'{pos}')

        self._image = cv.circle(self._image,pos,2,(255,0,0))

        cv.imshow(self._window, self._image)
        cv.waitKey(1)


if __name__ == '__main__':
    # create the node
    node = OdometryNode(node_name='odometry_node')
    # keep the process from terminating
    node.run()
    rospy.spin()