#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import socket
from cv_bridge import CvBridge
import cv2
import time


bridge = CvBridge()
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8084
sock.connect((host, port))


def image_callback(msg):
    sendData = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    start =  time.time()
    __, sendData = cv2.imencode('.jpg', sendData, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    print(time.time() - start)
    sendData = sendData.tobytes()
    size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    print("SENT", len(sendData))
    sock.sendall(size)
    sock.sendall(sendData)


rospy.init_node("image_subscriber")
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.spin()
sock.close()
