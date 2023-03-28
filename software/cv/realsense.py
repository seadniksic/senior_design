import sys
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
import socket
from cv_bridge import CvBridge
import math


bridge = CvBridge()
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8084
sock.connect((host, port))


window_title = "Stream"

input_size = 256
capture_width = 320
capture_height = 240


def detect_pose(frame):
                  
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        if video_capture.isOpened():
            try:
                window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE )
                # Window
                now = time.time()
                while count < 300:
                count += 1
                fps = 1 / (time.time() - now)
                if count != 1:
                rsum += fps
                now = time.time()

                ret_val, frame = video_capture.read()

                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                if not ret_val:
                continue
                else:
                frame.flags.writeable = False
                results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                #print(results.pose_landmarks)

    if results.pose_landmarks:
    mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
    cv2.putText(frame, f"{fps:.2}", (int(frame.shape[1] * 5 / 6), int(frame.shape[0] * 5 / 6)), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.imshow(window_title, frame)

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    pose = detect_pose(frame)
    
    # start =  time.time()
    # __, sendData = cv2.imencode('.jpg', sendData, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    # print(time.time() - start)
    # sendData = sendData.tobytes()
    # size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    # print("SENT", len(sendData))
    # sock.sendall(size)
    # sock.sendall(sendData)



rospy.init_node("image_subscriber")
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.spin()
sock.close()