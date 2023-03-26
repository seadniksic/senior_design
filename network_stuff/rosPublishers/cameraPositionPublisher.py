#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import socket, select, pickle

class CameraPositionPublisher(Node):
    def __init__(self):
        super().__init__('cameraPositionPublisher')
        self.cameraPositionPublisher = self.create_publisher(Point, 'cameraPositionStream', 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(("127.0.0.1", 8094))
        self.socket.listen(1)

        timerPeriod = 1 / 60
        self.create_timer(timerPeriod, self.cameraPositionTimerCallBack)
        self.client = None
    
    def cameraPositionTimerCallBack(self):
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
            
            pointData = pickle.load(msg)
            self.cameraPositionPublisher.publish(pointData)

        except Exception as e:
            print("Camera Position Publisher ran into " + str(e))
    
    def shutdownNode(self):
        if self.socket is not None:
            self.socket.close()
        if self.client is not None:
            self.client.close()

if __name__ == '__main__':
    try:
        rclpy.init()
        cameraPositionNode = CameraPositionPublisher()
        rclpy.spin(cameraPositionNode)
        cameraPositionNode.shutdownNode()
        cameraPositionNode.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

    finally:
        cameraPositionNode.shutdownNode()