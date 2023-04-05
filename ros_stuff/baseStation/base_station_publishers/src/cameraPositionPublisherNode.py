#!/usr/bin/env python

import rospy, pickle
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8MultiArray

class CameraPositionPublisher:
    def __init__(self):
        rospy.init_node('cameraPositionPublisher')
        self.publisher_ = rospy.Publisher('cameraPositionStream', Point, queue_size=10)
        self.subscriber_ = rospy.Subscriber('cameraPositionNetworkNodePublisher', UInt8MultiArray, self.callback)

        rospy.spin()
    
    def callback(self, msg):
        try:
            pointData = pickle.load(msg.data)
            self.publisher_.publish(pointData)

        except Exception as e:
            print("Camera Position Publisher ran into: " + str(e))

if __name__ == '__main__':
    try:
        cameraPositionNode = CameraPositionPublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass