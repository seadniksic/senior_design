#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField, Point, String
import sensor_msgs.point_cloud2 as pc2
import socket
import numpy as np
import pickle

class BigDaddyPublisher(Node):
    def __init__(self):
        super().__init__('big_daddy_publisher')
        self.imagePublisher_ = self.create_publisher(Image, 'imageStream', 10)
        self.pointCloudPublisher_ = self.create_publisher(PointCloud2, 'pointCloudStream', 10)
        self.positionPublisher_ = self.create_publisher(Point, 'cameraPositionStream', 10)
        self.statusPublisher_ = self.create_publisher(String, 'statusStream', 10)

        timerPeriod = 0.016666  # seconds

        self.imageTimer = self.create_timer(timerPeriod, self.imageTimerCallBack)
        self.pointCloudTimer = self.create_time(timerPeriod, self.pointCloudTimerCallBack)
        self.positionTimer = self.create_time(timerPeriod, self.positionTimerCallBack)
        self.statusTimer = self.create_time(timerPeriod, self.statusTimerCallBack)

        self.imageSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.imageSocket.bind(("127.0.0.1", 8089))
        self.imageClient = self.imageSocket.accept()

        self.pointCloudSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pointCloudSocket.bind(("127.0.0.1", 8090))
        self.pointCloudClient = self.pointCloudSocket.accept()

        self.positionSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.positionSocket.bind(("127.0.0.1", 8094))
        self.positionClient = self.positionSocket.accept()

        self.statusSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.statusSocket.bind(("127.0.0.1", 8091))
        self.statusClient = self.statusSocket.accept()

    def imageTimerCallBack(self):
        try:
            size = self.imageClient.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = self.imageClient.recv(size)
            while len(msg) < size:
                msg += self.imageClient.recv(size - len(msg))
            
            bridge = CvBridge()
            buffer = np.frombuffer(msg, np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            rosMsg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.imagePublisher_.publish(rosMsg)

        except Exception as e:
            print("ran into " + str(e))
    
    def pointCloudTimerCallBack(self):
        try:
            size = self.pointCloudClient.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = self.pointCloudClient.recv(1400)
            while len(msg) < size:
                msg += self.pointCloudClient.recv(min(1400, size - len(msg)))
            

            buffer = np.frombuffer(msg, np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            pointCloudMsg = PointCloud2()
            pointCloudMsg.header.stamp = rclpy.Time.now()
            pointCloudMsg.header.frame_id = "my_frame_id"
            pointCloudMsg.height = 1
            pointCloudMsg.width = image.shape[0]
            pointCloudMsg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('r', 12, PointField.FLOAT32, 1),
                PointField('g', 16, PointField.FLOAT32, 1),
                PointField('b', 20, PointField.FLOAT32, 1)]
            pointCloudMsg.is_bigendian = False
            pointCloudMsg.point_step = 24
            pointCloudMsg.row_step = pointCloudMsg.point_step * image.shape[0]
            pointCloudMsg.is_dense = True
            pointCloudMsg.data = image.astype(np.float32).tobytes()
            self.pointCloudPublisher_.publish(pointCloudMsg)
        except Exception as e:
            print("ran into " + str(e))
    
    def positionTimerCallBack(self):
        try:
            size = self.positionClient.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = self.positionClient.recv(1400)
            while len(msg) < size:
                msg += self.positionClient.recv(min(1400, size - len(msg)))
        
            pointData = pickle.load(msg)
            self.positionPublisher_.publish(pointData)

        except Exception as e:
            print("ran into " + str(e))
    
    def statusTimerCallBack(self):
        try:
            size = self.statusClient.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = self.statusClient.recv(1400)
            while len(msg) < size:
                msg += self.statusClient.recv(min(1400, size - len(msg)))
            
            self.statusPublisher_.publish(str(msg))
        
        except Exception as e:
            print("ran into " + e)
    
def main(args=None):
    rclpy.init(args=args)

    rosDaddy = BigDaddyPublisher()

    rclpy.spin(rosDaddy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rosDaddy.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()