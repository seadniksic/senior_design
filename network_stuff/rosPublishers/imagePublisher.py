#!/usr/bin/env python

import cv2, socket
import numpy as np
import rclpy as rospy


def publisher(data):
      
    # define the actions the publisher will make
    pub = rospy.Publisher('/roverLiveStream',
                          cv2.Mat, queue_size=10)
    # initialize the publishing node
    rospy.init_node('imageCapture', anonymous=True)
      
    # define how many times per second
    # will the data be published
    # let's say 10 times/second or 10Hz
    rate = rospy.Rate(10)
    # to keep publishing as long as the core is running
    while not rospy.is_shutdown():
        rospy.loginfo("W")
          
        # publish the data to the topic using publish()
        pub.publish(data)
          
        # keep a buffer based on the rate defined earlier
        rate.sleep()

if __name__ == '__main__':
    # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # host = "127.0.0.1"
    # port = 8089

    # sock.bind((host, port))
    # sock.listen(1)
    # client, addr = sock.accept()

    # while True:
    #     byteSize = client.recv(1400).from_bytes(8, byteorder='little', signed=False)
    #     msg = client.recv(1400)
    #     while(len(msg) < byteSize):
    #         msg += client.recv(1400)
        
        #publish to ros
    msg = cv2.imread("testImage_3.jpg")
    # nparr = np.frombuffer(msg, np.uint8)
    # img_np = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
    try:
        publisher(msg)
    except rospy.ROSInterruptException:
        pass