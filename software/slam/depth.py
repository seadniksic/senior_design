#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import socket

from cv_bridge import CvBridge
import cv2
import time
import numpy as np



class ImageTransmitter:
    def __init__(self):
        self.bridge = CvBridge()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(("127.0.0.1", 8084))
        self.oldImage = np.zeros((480, 640, 3), np.uint8)
    def image_callback(self, msg):
        sendData = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # start =  time.time()
        __, sendData = cv2.imencode('.jpg', sendData, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        # print(time.time() - start)
        sendData = sendData.tobytes()
        size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
        # print("SENT", len(sendData))
        self.sock.sendall(size)
        self.sock.sendall(sendData)    

if __name__ == '__main__':
    imageSender = ImageTransmitter()
    rospy.init_node("image_subscriber")
    rospy.Subscriber("/camera/color/image_raw", Image, imageSender.image_callback)
    rospy.spin()
    imageSender.sock.close()

