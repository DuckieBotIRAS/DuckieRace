#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import yaml
import cv2
import numpy as np
from cv_bridge import CvBridge
import tkinter as tk

from PIL import Image, ImageTk

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        with open('packages/followlane/config/detect_lane.yaml','r') as f:
            text = f.read()
        self.conf = yaml.safe_load(text)

        self.create_window()

        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self,msg):

        self.update_conf()

        #print('started Method')
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        #print(f'converted image')
        # display frame
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #print(f'loaded image {image.shape}')
        if self.selected.get() == 'lane_image':

            x_alt = 0
            y_alt = 0
            for point in ['top_left','top_right','bottom_left','bottom_right','top_left']:
                x = self.conf['lane_image'][f'{point}_x']
                y = self.conf['lane_image'][f'{point}_y']

                if x_alt != 0 or y_alt != 0:
                    image = cv2.line(image,(x_alt,y_alt),(x,y),(255,255,255),2 )

                x_alt = x
                y_alt = y

        else:
            hl = self.conf[self.selected.get()]['hl']
            hh = self.conf[self.selected.get()]['hh'] 
            sl = self.conf[self.selected.get()]['sl']
            sh = self.conf[self.selected.get()]['sh']
            vl = self.conf[self.selected.get()]['vl']
            vh = self.conf[self.selected.get()]['vh'] 

            image = cv2.inRange(image, 
                            (hl,sl,vl), 
                            (hh,sh,vh),)

        #update image in window
        image = ImageTk.PhotoImage(Image.fromarray(image))
        self.panel.configure(image=image)
        self.panel.image = image

    def update_conf(self):
        for val in self.conf[self.selected.get()]:
            name = f'{self.selected.get()}_{val}'
            self.conf[self.selected.get()][val] = self.sliders[name].get()

    
    def print_conf(self):
        text = yaml.safe_dump(self.conf)
        print(f'#############\n{text}\n#############')

    def change_menue(self,*args):
        self.slider_frame.pack_forget()
        print(f'selected menue : {self.selected.get()}')
        self.slider_frame = self.slider_frames[self.selected.get()]
        self.slider_frame.pack()
        self.print_conf()

    def create_window(self):
        self._root = tk.Tk()

        #Add Image For start only Black
        img = ImageTk.PhotoImage(Image.fromarray(np.zeros([480,640,3], np.uint8)))

        self.panel = tk.Label(self._root, image = img)
        self.panel.pack(side='bottom')

        #Add drop down menu
        options = [s for s in self.conf]

        self.selected = tk.StringVar(self._root)
        self.selected.set(options[0])
        self.selected.trace("w", lambda *args : self.change_menue(*args))

        self.dropdown = tk.OptionMenu(self._root, self.selected, *options)
        self.dropdown.pack(side='top')

        #Add sliders
        self.sliders = {}
        self.slider_frames = {}
        for option in options:
            frame = tk.Frame(self._root)
            for val in self.conf[option]:
                name = f'{option}_{val}'

                if option == 'lane_image':
                    self.sliders[name] = tk.Scale(frame, from_=-100, to=700,orient='horizontal',label=val)
                else:
                    self.sliders[name] = tk.Scale(frame, from_=0, to=255,orient='horizontal',label=val)
                self.sliders[name].set(self.conf[option][val])
                self.sliders[name].pack(side='left')

            self.slider_frames[option]  = frame

        self.slider_frame = self.slider_frames[self.selected.get()]
        self.slider_frame.pack()

    def run(self):
        self._root.mainloop()
        self.print_conf()
        rospy.signal_shutdown('User endet Programm')


if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    node.run()
    rospy.spin()