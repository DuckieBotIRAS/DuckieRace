#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml


from duckietown.dtros import DTROS, NodeType
from my_msg.msg import Intersection

class DetectIntersectionNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectIntersectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)


        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._conf_topic = f"/{self._vehicle_name}/conf"
        
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindIntersection, queue_size = 1)
        self.sub_config = rospy.Subscriber(self._conf_topic, String, self.cbUpdateConf, queue_size = 1)

        self.pub_Intersection = rospy.Publisher(f'/{self._vehicle_name}/detect/Intersection', Intersection, queue_size = 1)
        self._window = 'Detected Intersection'

        self.intersection = False
        self.right = False
        self.left = False
        self.straight = False

        self.is_running = False
        self.counter = 0

    def cbFindIntersection(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1   
            return
        else:
            self.counter += 1


        if self.is_running:
            return
        self.is_running = True

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask_red = cv2.inRange(hsv, 
                           (self.hue_red_l,self.saturation_red_l, self.lightness_red_l), 
                           (self.hue_red_h,self.saturation_red_h, self.lightness_red_h),)


        stop_line_x = int(len(image[0]) / 2)
        stop_line_y = len(image) - 21

        radius = 50

        print(np.sum(np.where(mask_red[stop_line_y -radius :  stop_line_y +radius,stop_line_x -radius:  stop_line_x + radius]  != 0, 1, 0)))

        if 100 > np.sum(np.where(mask_red[stop_line_y -radius :  stop_line_y +radius,stop_line_x -radius:  stop_line_x + radius]  != 0, 1, 0)):
            image = cv2.circle(image,(stop_line_x , stop_line_y),radius,(0,0,255))
            self.intersection = False
        else:
            image = cv2.circle(image,(stop_line_x , stop_line_y),radius,(0,255,0))
            self.intersection = True

        #print(f'{self.red_marker_left_y -radius} : {self.red_marker_left_y +radius} -- {self.red_marker_left_x -radius} : {self.red_marker_left_x +radius}')
        #print(f'left : {np.sum(np.where(mask_red[self.red_marker_left_y -radius :  self.red_marker_left_y +radius,self.red_marker_left_x -radius :  self.red_marker_left_x +radius]  != 0,1,0))}')
        #cv2.imshow('Test',image[self.red_marker_left_y -radius :  self.red_marker_left_y +radius,0 :  self.red_marker_left_x +radius])
        
        if 20 > np.sum(np.where(mask_red[self.red_marker_left_y -radius :  self.red_marker_left_y +radius,np.max([0,self.red_marker_left_x -radius]) :  self.red_marker_left_x +radius]  != 0,1,0)):
            image = cv2.circle(image,(self.red_marker_left_x, self.red_marker_left_y),radius,(0,0,255))
            self.left = False
        else:
            image = cv2.circle(image,(self.red_marker_left_x, self.red_marker_left_y),radius,(0,255,0))
            self.left = True

        if 20 > np.sum(np.where(mask_red[self.red_marker_right_y -radius :  self.red_marker_right_y +radius,self.red_marker_right_x -radius :  self.red_marker_right_x +radius]  != 0,1,0)):
            image = cv2.circle(image,(self.red_marker_right_x, self.red_marker_right_y),radius,(0,0,255))
            self.right = False
        else:
            image = cv2.circle(image,(self.red_marker_right_x, self.red_marker_right_y),radius,(0,255,0))
            self.right = True

        if 20 > np.sum(np.where(mask_red[self.red_marker_front_y -radius :  self.red_marker_front_y +radius,self.red_marker_front_x -radius :  self.red_marker_front_x +radius]  != 0,1,0)):
            image = cv2.circle(image,(self.red_marker_front_x, self.red_marker_front_y),radius,(0,0,255))
            self.straight = False
        else:
            image = cv2.circle(image,(self.red_marker_front_x, self.red_marker_front_y),radius,(0,255,0))
            self.straight = True
        
        #publish detected intersections
        msg = Intersection()
        msg.straight = self.straight
        msg.right = self.right
        msg.left = self.left
        msg.intersection = self.intersection
        self.pub_Intersection.publish(msg)
        
        cv2.imshow(self._window, image)
        #cv2.imshow('red', mask_red)
        cv2.waitKey(1)
        self.is_running = False

        
    def load_conf(self,path):

        with open(path,'r') as f:
            conf_yml = f.read()
        
        self.update_conf(conf_yml)

    def cbUpdateConf(self,conf_msg):
        self.update_conf(conf_msg.data)
    
    def update_conf(self,conf_yml):
        self.conf = yaml.safe_load(conf_yml)
        
        self.hue_red_l = self.conf['red']['hl']
        self.hue_red_h = self.conf['red']['hh']
        self.saturation_red_l =  self.conf['red']['sl']
        self.saturation_red_h =  self.conf['red']['sh']
        self.lightness_red_l =  self.conf['red']['vl']
        self.lightness_red_h =  self.conf['red']['vh']

        self.red_marker_left_x = self.conf['turn_detect']['left_x']
        self.red_marker_left_y = self.conf['turn_detect']['left_y']
        self.red_marker_right_x = self.conf['turn_detect']['right_x']
        self.red_marker_right_y = self.conf['turn_detect']['right_y']
        self.red_marker_front_x = self.conf['turn_detect']['front_x']
        self.red_marker_front_y = self.conf['turn_detect']['front_y']

    
            
        
if __name__ == '__main__':

    node = DetectIntersectionNode(node_name='detect_Intersection_node')
    rospy.spin()
