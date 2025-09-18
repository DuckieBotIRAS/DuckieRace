#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, String, UInt8
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml


from duckietown.dtros import DTROS, NodeType

class DetectLaneNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectLaneNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)


        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._conf_topic = f"/{self._vehicle_name}/conf"
        
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)
        self.sub_config = rospy.Subscriber(self._conf_topic, String, self.cbUpdateConf, queue_size = 1)

        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)
        self.pub_parking = rospy.Publisher(f'/{self._vehicle_name}/detect/parking', UInt8, queue_size = 1)

        self._window = 'Detected Lane'
        self._crop_im_size = 500
        #cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.is_running = False
        self.counter = 0

    def crop_img(self,img):
        img = img.copy()
        print(img.shape)

        pts1 = np.float32([
            [self.conf['lane_image']['top_left_x'],     self.conf['lane_image']['top_left_y']],
            [self.conf['lane_image']['top_right_x'],    self.conf['lane_image']['top_right_y']],
            [self.conf['lane_image']['bottom_right_x'], self.conf['lane_image']['bottom_right_y']],
            [self.conf['lane_image']['bottom_left_x'],  self.conf['lane_image']['bottom_left_y']],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(self._crop_im_size,self._crop_im_size))


    def get_x_for_driving(self, mask, distance, no_lane_value):
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=5, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        _,th1 = cv2.threshold(grad,127,255,cv2.THRESH_BINARY)

        a = []
        for row in range(distance-int(self._crop_im_size*0.1), distance+int(self._crop_im_size*0.1)):
            if np.where(th1[row] == 255)[0].size == 0:
                continue
            else:
                a.append(np.where(th1[row] == 255)[0][-1])
        if len(a) >0:
            return np.mean(a),th1
        else:
            return no_lane_value,th1 #1300 for turning right when no white Line detected sonst 900


    def cbFindLane(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1   
            return
        else:
            self.counter += 1
        

        if self.is_running:
            return
        self.is_running = True

        # Write your own Code for Lane detection here
        # This is only a basic example to get some inspiration from

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.crop_img(cv_image)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)

        center_white,th_white = self.get_x_for_driving(mask_white,int(len(img)*0.75),len(img[0]) * 0.9)
        center_yellow,th_yellow = self.get_x_for_driving(mask_yellow,int(len(img)*0.75),len(img[0]) *0)
        #center_white = np.mean(np.where(mask_white != 0))
        #center_yellow = np.mean(np.where(mask_yellow != 0))



        #if np.isnan(center_white):
        #    center_white = 90

        #if np.isnan(center_yellow):
        #    center_yellow = 10

        lane_center = (center_white + center_yellow) / 2
        msg_error = Float64()
        msg_error.data = lane_center / len(img) * 2 - 1
        self.pub_lane.publish(msg_error)
        print(msg_error.data)

        

        idx = np.where(th_white != 0)
        img[idx] = (255,0,255)

        self.detect_parking(idx)    

        idx = np.where(th_yellow != 0)
        img[idx] = (255,255,0)

        #img = np.where(th_white == 255, (255,255,255),img)
        #img = np.where(th_yellow == 255, (0,255,255),img)
        
        img = cv2.circle(img,(int(lane_center),int(len(img)*0.75)),20,(0,255,0),5)
        img = cv2.circle(img,(int(center_white),int(len(img)*0.75)),20,(255,255,255),5)
        img = cv2.circle(img,(int(center_yellow),int(len(img)*0.75)),20,(0,255,255),5)

        cv2.imshow(self._window, img)
        #cv2.imshow('white', mask_white)
        #cv2.imshow('yellow', mask_yellow)
        cv2.waitKey(1)
        self.is_running = False

    def detect_parking(self,idx_white):
        y_prev = self._crop_im_size * 0.5
        gap = 0
        for y in idx_white[0]:
            if y - y_prev > 10:
                gap += 1
            if y > y_prev:
                y_prev = y

        if gap >= 2:
            print(f'parking {gap} gaps')
            self.pub_parking.publish(UInt8(1))
            return

        print(f'not parking {gap} gaps')
        self.pub_parking.publish(UInt8(0))

        
    def load_conf(self,path):

        with open(path,'r') as f:
            conf_yml = f.read()
        
        self.update_conf(conf_yml)

    def cbUpdateConf(self,conf_msg):
        self.update_conf(conf_msg.data)
    
    def update_conf(self,conf_yml):
        self.conf = yaml.safe_load(conf_yml)

        self.hue_white_l = self.conf['white']['hl']
        self.hue_white_h = self.conf['white']['hh']
        self.saturation_white_l = self.conf['white']['sl']
        self.saturation_white_h = self.conf['white']['sh']
        self.lightness_white_l = self.conf['white']['vl']
        self.lightness_white_h = self.conf['white']['vh']
        
        self.hue_yellow_l = self.conf['yellow']['hl']
        self.hue_yellow_h = self.conf['yellow']['hh']
        self.saturation_yellow_l =  self.conf['yellow']['sl']
        self.saturation_yellow_h =  self.conf['yellow']['sh']
        self.lightness_yellow_l =  self.conf['yellow']['vl']
        self.lightness_yellow_h =  self.conf['yellow']['vh']
        
        self.hue_duck_l = self.conf['duck']['hl']
        self.hue_duck_h = self.conf['duck']['hh']
        self.saturation_duck_l =  self.conf['duck']['sl']
        self.saturation_duck_h =  self.conf['duck']['sh']
        self.lightness_duck_l =  self.conf['duck']['vl']
        self.lightness_duck_h =  self.conf['duck']['vh']
            
        
if __name__ == '__main__':

    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()
