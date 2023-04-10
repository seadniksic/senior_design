#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8MultiArray
import cv2
import time


bridge = CvBridge()
newPub = rospy.Publisher("cameraFeedNetworkNodeSubscriber", UInt8MultiArray, 5)

def image_callback(msg):
    sendData = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    start =  time.time()
    __, sendData = cv2.imencode('.jpg', sendData, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    messagePayload = UInt8MultiArray()
    messagePayload.data = sendData.tobytes()
    newPub.publish(messagePayload)


rospy.init_node("image_subscriber")
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.spin()
