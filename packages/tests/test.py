
import numpy as np
import tkinter as tk
import cv2
import yaml

from PIL import Image, ImageTk

root = tk.Tk()
img = ImageTk.PhotoImage(Image.fromarray(np.zeros([480,640,3], np.uint8)))

panel = tk.Label(root, image = img)

panel.pack(side="bottom", fill="both", expand="yes")



with open('packages/followlane/config/detect_lane.yaml','r') as f:
        text = f.read()
        conf = yaml.safe_load(text)

for c in ['h','s','v']:
    w = tk.Scale(root, from_=0, to=255,orient='horizontal',label=f'{c}_low')
    w.pack(side='left')
    w = tk.Scale(root, from_=0, to=255,orient='horizontal',label=f'{c}_high')
    w.pack(side='left')

print(w.get())
root.mainloop()
