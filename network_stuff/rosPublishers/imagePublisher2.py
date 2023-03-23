import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from std_msgs.msg import String
import socket
import numpy as np
import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'imageStream', 10)
        timer_period = 0.016666  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "127.0.0.1"
        port = 8089

        self.sock.bind((host, port))
        self.sock.listen(1)
        self.client, addr = self.sock.accept()
        self.readSockets = [self.client]


    def timer_callback(self):
        size = self.client.recv(8)
        size = int.from_bytes(size, 'little')
        if size <= 0:
            return
        msg = self.client.recv(1400)
        while len(msg) < size:
            msg += self.client.recv(min(1400, size - len(msg)))
        

        bridge = CvBridge()
        buffer = np.frombuffer(msg, np.uint8)
        image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        rosMsg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.publisher_.publish(rosMsg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()