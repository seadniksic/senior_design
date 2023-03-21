#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8084
sock.connect((host, port))


def image_callback(msg):
    sendData = msg.tobytes()
    size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    sock.sendall(size)
    sock.sendall(sendData)


rospy.init_node("image_subscriber")
rospy.Subscriber("/camera/image", Image, image_callback)
rospy.spin()
sock.close()
