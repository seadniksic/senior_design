#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import PointCloud2, PointField
import socket, select
import numpy as np

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointCloudPublisher')
        self.pointCloudPublisher = self.create_publisher(PointCloud2, 'pointCloudStream', 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(("127.0.0.1", 8090))
        self.socket.listen(1)

        timerPeriod = 1 / 60
        self.create_timer(timerPeriod, self.pointCloudTimerCallBack)
        self.client = None
    
    def pointCloudTimerCallBack(self):
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

            self.pointCloudPublisher.publish(pointCloudMsg)

        except Exception as e:
            print("Point Cloud Publisher ran into " + str(e))
    
    def shutdownNode(self):
        if self.socket is not None:
            self.socket.close()
        if self.client is not None:
            self.client.close()

if __name__ == '__main__':
    try:
        rclpy.init()
        pointCloudNode = PointCloudPublisher()
        rclpy.spin(pointCloudNode)
        pointCloudNode.shutdownNode()
        pointCloudNode.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

    finally:
        pointCloudNode.shutdownNode()