#!/usr/bin/env python

import rospy, cv2, socket, select
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class ImagePublisher:
    def __init__(self):
        rospy.init_node('imagePublisher')
        self.publisher_ = rospy.Publisher('imageFeedStream', Image, queue_size=10)
        timer_period = rospy.Duration.from_sec(0.016666)  # seconds
        self.timer = rospy.Timer(timer_period, self.timer_callback)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(("127.0.0.1", 8089))
        self.socket.listen(1)

        self.client = None
        self.bridge = CvBridge()
    
    def timer_callback(self):
        try:
            if self.client is None:
                readSocket, writeSocket, errorSocket = select.select([self.socket], [], [], 0)
                if len(readSocket) == 0:
                    return
                self.client, addr = self.socket.accept()
            readSocket, writeSocket, errorSocket = select.select([self.client], [], [], 0)
            if len(readSocket) == 0:
                return
            size = self.client.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = self.client.recv(size)
            while len(msg) < size:
                msg += self.client.recv(size - len(msg))
            
            buffer = np.frombuffer(msg, np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            rosMsg = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.publisher_.publish(rosMsg)

        except Exception as e:
            print("Image Publisher ran into " + str(e))
    
    def shutdownNode(self):
        if self.socket is not None:
            self.socket.close()
        if self.client is not None:
            self.client.close()

if __name__ == '__main__':
    try:
        imageNode = ImagePublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass

    finally:
        imageNode.shutdownNode()