'''
Author:  Sead Niksic
Code inspired by: https://github.com/jetsonhacks/USB-Camera/blob/main/usb-camera-simple.py && https://google.github.io/mediapipe/solutions/pose#python-solution-api
&& tf documentation https://tfhub.dev/google/lite-model/movenet/multipose/lightning/tflite/float16/1
&& poseDrawing from https://github.com/nicknochnack/MultiPoseMovenetLightning/blob/main/MultiPose%20MoveNet%20Tutorial.ipynb


'''

import sys
import cv2
import numpy as np
import time

import math


window_title = "Stream"



input_size = 256
capture_width = 320
capture_height = 240



def detect_pose():

    camera_id = "/dev/video0"

    # webcams -> V4L2, stereo -> gstreamer
    video_capture = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)

    rsum = 0
    count = 0
    if video_capture.isOpened():
      try:
          window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE )
          # Window
          now = time.time()
          while count < 150:
              count += 1
              fps = 1 / (time.time() - now)
              if count != 1:
                  rsum += fps

              now = time.time()
                  
              ret_val, frame = video_capture.read()
              #print(frame.shape)

              # Check to see if the user closed the window
              # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
              # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
              if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                  if not ret_val:
                      continue
                      
                  else:
                      
                      print(fps)
                      #cv2.putText(frame, f"{fps:.4}", (int(frame.shape[1] * 5 / 6), int(frame.shape[0] * 5 / 6)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                      cv2.imshow(window_title, frame)
                    
              else:
                  break
              
              keyCode = cv2.waitKey(10) & 0xFF
              # Stop the program on the ESC key or 'q'
              if keyCode == 27 or keyCode == ord('q'):
                  break
              
          print(f"Average FPS: {rsum / 150}")
      finally:
          video_capture.release()
          cv2.destroyAllWindows()
    else:
        print("Unable to open camera")



if __name__ == "__main__":

    detect_pose()
