#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import time
import threading



class DetectDuckieNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectDuckieNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        self._model = YOLO("packages/followlane/assets/model.pt") 

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbDetectObjects, queue_size = 1)
        self._yolo_topic = f"/{self._vehicle_name}/detect/duckie/image"
        self.pup_image = rospy.Publisher(self._yolo_topic,Image,queue_size = 1)
        self._duckie_topic = f"/{self._vehicle_name}/detect/duckie"
        self.pup_duckie = rospy.Publisher(self._duckie_topic,Float64,queue_size = 1)

        self.counter = 0
        self.bridge = CvBridge()
        self.is_running = False
        self.image = None


    def cbDetectObjects(self,image_msg):
        #if self.counter % 3 != 0:
        #    self.counter += 1
        #    return
        #else:
        #    self.counter += 1
        
        print(f'is running : {self.is_running}')
        if self.is_running:
            return
        
        threading.Thread(target = self.predict_duckies,args=[image_msg]).start()
        #self.predict_duckies(np_arr)
        
    def predict_duckies(self,image_msg):

        self.is_running = True
        print('started')
        t = time.time()

        
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self._model(cv_image) #, classes=)
        self.image = draw_bounding_boxes(results,cv_image)


        print('done')
        self.is_running = False

        print(time.time() - t)
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print(f'showing image {self.image is None}')
            if not self.image is None:
                cv2.imshow('duckie detection',self.image)
                cv2.waitKey(1)
            rate.sleep()

        


def draw_bounding_boxes(results,img):
    for result in results:
        for box in result.boxes:
            cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                          (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), 5)
            cv2.putText(img, f"{result.names[int(box.cls[0])]}",
                        (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 5)
    return img

if __name__ == '__main__':

    node = DetectDuckieNode(node_name='detect_duckie_node')
    node.run()
    rospy.spin()