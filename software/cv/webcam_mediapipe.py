'''
Author:  Sead Niksic
Code inspired by: https://github.com/jetsonhacks/USB-Camera/blob/main/usb-camera-simple.py && https://google.github.io/mediapipe/solutions/pose#python-solution-api

'''



import sys
import cv2
import mediapipe as mp
import time
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose


window_title = "USB Camera"

def show_camera():
    # ASSIGN CAMERA ADDRESS HERE
    camera_id = "/dev/video0"
    # Full list of Video Capture APIs (video backends): https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
    # For webcams, we use V4L2
    video_capture = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    rsum = 0
    count = 0
    with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
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
                    #print(f"fps: {fps}"
                    now = time.time()
                    
                    ret_val, frame = video_capture.read()

                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                    if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                        if not ret_val:
                            continue
                    
                        
                        else:
                            frame.flags.writeable = False
                            results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                            
                            if not results.pose_landmarks:
                                cv2.imshow(window_title, frame)
                                continue
                            
                            #frame.flags.writeable = True
                            #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    
                            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
                    #show frame
                            #print(frame.shape)
                            cv2.putText(frame, f"{fps:.2}", (int(frame.shape[1] * 5 / 6), int(frame.shape[0] * 5 / 6)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
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

    show_camera()
