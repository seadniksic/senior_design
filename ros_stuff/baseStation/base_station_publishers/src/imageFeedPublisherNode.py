#!/usr/bin/env python

import rospy, cv2
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class ImagePublisher:
    def __init__(self):
        rospy.init_node('imagePublisher')
        self.publisher_ = rospy.Publisher('imageFeedStream', Image, queue_size=10)
        self.subscriber_ = rospy.Subscriber('cameraFeedNetworkNodePublisher', UInt8MultiArray, self.callback)

        self.bridge = CvBridge()
        rospy.spin()
    
    def callback(self, msg):
        try: 
            data = msg.data
            buffer = np.frombuffer(data, np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            rosMsg = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.publisher_.publish(rosMsg)

        except Exception as e:
            print("Image Publisher ran into: " + str(e))

if __name__ == '__main__':
    try:
        imageNode = ImagePublisher()

    except KeyboardInterrupt:
        pass

