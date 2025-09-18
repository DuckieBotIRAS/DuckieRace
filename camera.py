#!/usr/bin/env python3

import os
import yaml
import cv2
import numpy as np
import tkinter as tk
from threading import Thread


from PIL import Image, ImageTk

class CameraReaderNode():

    def __init__(self, node_name):
        # static parameters
        self._vehicle_name ='tick' #os.environ['VEHICLE_NAME']
        
        with open('packages/followlane/config/detect_lane.yaml','r') as f:
            text = f.read()
        self.conf = yaml.safe_load(text)

        self.create_window()

        self.is_running = False

    def callback(self,img):

        if self.is_running:
            return
        self.is_running = True
        topic = self.selected.get()

        print(f'call back called')
        self.update_conf()
        print(f'config updated')

        #print('started Method')
        # convert JPEG bytes to CV image
        image = img
        #print(f'converted image')
        # display frame
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #print(f'loaded image {image.shape}')
        print(f'read image {image.shape}')
        print(f'read image {image.size}')

        if topic == 'lane_image':

            x_alt = 0
            y_alt = 0
            for point in ['top_left','top_right','bottom_left','bottom_right','top_left']:
                x = self.conf['lane_image'][f'{point}_x']
                y = self.conf['lane_image'][f'{point}_y']

                if x_alt != 0 or y_alt != 0:
                    image = cv2.line(image,(x_alt,y_alt),(x,y),(255,255,255),2 )

                x_alt = x
                y_alt = y

        elif topic == 'pid_regler'  or topic == 'turn_control':
            pass

        elif topic == 'turn_detect':
            image = cv2.circle(image,(self.conf[topic]['left_x'],  self.conf[topic]['left_y']),20,(255,0,0))
            image = cv2.circle(image,(self.conf[topic]['right_x'], self.conf[topic]['right_y']),20,(255,0,0))
            image = cv2.circle(image,(self.conf[topic]['front_x'], self.conf[topic]['front_y']),20,(255,0,0))

        elif topic == 'duckie_areas':
            for area in ['intersection_front','intersection_left','intersection_right','parkingt','overtaking_1','overtaking_2','overtaking_3']:
                image = cv2.circle(image,(self.conf[topic][f'{area}_x'], self.conf[topic][f'{area}_y']),self.conf[topic][f'{area}_size'],(255,0,0))
            

        else:
            hl = self.conf[topic]['hl']
            hh = self.conf[topic]['hh'] 
            sl = self.conf[topic]['sl']
            sh = self.conf[topic]['sh']
            vl = self.conf[topic]['vl']
            vh = self.conf[topic]['vh'] 

            image = cv2.inRange(image, 
                            (hl,sl,vl), 
                            (hh,sh,vh),)

        print('showing image')
        #update image in window
        image = ImageTk.PhotoImage(Image.fromarray(image))
        self.panel.configure(image=image)
        self.panel.image = image

        print('done')
        self.is_running = False

    def update_conf(self):
        selected = self.selected.get()
        for val in self.conf[selected]:
            name = f'{selected}_{val}'
            try:
                self.conf[selected][val] = self.sliders[name].get()
            except:
                print('bad conding should not happen')  

    
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
        print('creating windows: should only be called once')

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
            slider_row_frame = tk.Frame(frame)
            for i,val in enumerate(self.conf[option]):

                if i % 3 == 0:
                    slider_row_frame.pack()
                    slider_row_frame = tk.Frame(frame)
                
                name = f'{option}_{val}'

                if option == 'lane_image' or option == 'turn_detect' or option == 'duckie_areas':
                    self.sliders[name] = tk.Scale(slider_row_frame, from_=-100, to=700,orient='horizontal',label=val, length=150)
                elif option == 'pid_regler' or option == 'turn_control':
                    self.sliders[name] = tk.Scale(slider_row_frame, from_=0, to=100,orient='horizontal',label=val, length=150)
                else:
                    self.sliders[name] = tk.Scale(slider_row_frame, from_=0, to=255,orient='horizontal',label=val, length=150)
                self.sliders[name].set(self.conf[option][val])

                self.sliders[name].pack(side='left')

            slider_row_frame.pack()
            self.slider_frames[option]  = frame

        self.slider_frame = self.slider_frames[self.selected.get()]
        self.slider_frame.pack()

    def run(self):
        self._root.mainloop()
        self.print_conf()



import time
def show_img(node):
    while(True):
        img = cv2.imread('/home/duckie5/Pictures/Screenshots/Screenshot from 2025-03-18 17-17-50.png')
        #cv2.imshow('image',img)
        #cv2.waitKey(1)
        node.callback(img)
        time.sleep(0.1)



node = CameraReaderNode(node_name='camera_reader_node')

Thread(target=show_img,args=(node,)).start()
node.run()

