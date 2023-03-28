# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2
import mediapipe as mp
import time
#import tensorflow as tf
#import numpy as np


#mpPose = mp.solutions.pose
#pose = mpPose.Pose()
#mpDraw = mp.solutions.drawing_utils

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )




def show_camera():
    window_title = "CSI Camera"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    #print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    while(not video_capture.isOpened()):
    	continue
    try:
      window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
      while True:
      	ret_val, frame = video_capture.read()
      	#imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
      	#scale_percent = 20 # percent of original size
      	#width = int(img.shape[1] * scale_percent / 100)
      	#height = int(img.shape[0] * scale_percent / 100)
      	#dim = (width, height)
      	#img  = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
      	#results = pose.process(imgRGB)
      	#print(results.pose_landmarks)
      	#if results.pose_landmarks:
      		#mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
      	#for id, lm in enumerate(results.pose_landmarks.landmark):
      		#h, w,c = img.shape
      		#print(id, lm)
      		#cx, cy = int(lm.x*w), int(lm.y*h)
      		#cv2.circle(img, (cx, cy), 5, (255,0,0), cv2.FILLED)
      	#cTime = time.time()
      	#fps = 1/(cTime-pTime)
      	#pTime = cTime
      	#cv2.putText(img, str(int(fps)), (50,50), cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0), 3)
      	if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
      		cv2.imshow(window_title, frame)
      	else:
      		break
      	keyCode = cv2.waitKey(10) & 0xFF
      	if keyCode == 27 or keyCode == ord('q'):
      		break
    finally:
    	video_capture.release()
    	cv2.destroyAllWindows()


if __name__ == "__main__":
    show_camera()
