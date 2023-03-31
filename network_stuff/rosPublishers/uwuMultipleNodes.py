#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import String
import socket, select
import numpy as np
import pickle

def imagePublisher(updateRate, serverSocket, client):
    node = rclpy.create_node('imageStreamNode')
    publisher = node.create_publisher(Image, 'imageStreamTopic', 10)
    rate = node.create_rate(updateRate)

    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.bind(("127.0.0.1", 8089))
    serverSocket.listen(1)

    bridge = CvBridge()

    while rclpy.ok():
        try:
            if client is None:
                readSocket, writeSocket, errorSocket = select.select([serverSocket], [], [], 0)
                if len(readSocket) == 0:
                    return
                client, addr = serverSocket.accept()
            size = client.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = client.recv(size)
            while len(msg) < size:
                msg += client.recv(size - len(msg))
            
            buffer = np.frombuffer(msg, np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            rosMsg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
            publisher.publish(rosMsg)
            rate.sleep()

        except Exception as e:
            print("Image publisher ran into " + str(e))
        
def pointCloudPublisher(updateRate, serverSocket, client):
    node = rclpy.create_node('pointCloudStreamNode')
    publisher = node.create_publisher(PointCloud2, 'pointCloudStreamTopic', 10)
    rate = node.create_rate(updateRate)

    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.bind(("127.0.0.1", 8090))
    serverSocket.listen(1)

    while rclpy.ok():
        try:
            if client is None:
                readSocket, writeSocket, errorSocket = select.select([serverSocket], [], [], 0)
                if len(readSocket) == 0:
                    return
                client, addr = serverSocket.accept()
            size = client.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = client.recv(size)
            while len(msg) < size:
                msg += client.recv(size - len(msg))
            
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
            publisher.publish(pointCloudMsg)

            rate.sleep()

        except Exception as e:
            print("Point Cloud publisher ran into " + str(e))

def positionPublisher(updateRate, serverSocket, client):
    node = rclpy.create_node('cameraPositionStreamNode')
    publisher = node.create_publisher(PointCloud2, 'cameraPositionStreamTopic', 10)
    rate = node.create_rate(updateRate)

    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.bind(("127.0.0.1", 8094))
    serverSocket.listen(1)

    while rclpy.ok():
        try:
            if client is None:
                readSocket, writeSocket, errorSocket = select.select([serverSocket], [], [], 0)
                if len(readSocket) == 0:
                    return
                client, addr = serverSocket.accept()
            size = client.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = client.recv(size)
            while len(msg) < size:
                msg += client.recv(size - len(msg))
            
            pointData = pickle.load(msg)
            publisher.publish(pointData)

            rate.sleep()

        except Exception as e:
            print("Camera Position publisher ran into " + str(e))

def statusPublisher(updateRate, serverSocket, client):
    node = rclpy.create_node('roverStatusNode')
    publisher = node.create_publisher(PointCloud2, 'roverStatusNodeTopic', 10)
    rate = node.create_rate(updateRate)

    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.bind(("127.0.0.1", 8091))
    serverSocket.listen(1)

    while rclpy.ok():
        try:
            if client is None:
                readSocket, writeSocket, errorSocket = select.select([serverSocket], [], [], 0)
                if len(readSocket) == 0:
                    return
                client, addr = serverSocket.accept()
            size = client.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = client.recv(size)
            while len(msg) < size:
                msg += client.recv(size - len(msg))
            
            publisher.publish(msg)

            rate.sleep()

        except Exception as e:
            print("Camera Position publisher ran into " + str(e))

if __name__ == '__main__':
    try:
        rclpy.init()

        imageRate = 60
        imageSocket = None
        imageClient = None
        imagePublisher(imageRate, imageSocket, imageClient)

        pointCloudRate = 60
        pointCloudSocket = None
        pointCloudClient = None
        pointCloudPublisher(pointCloudRate, pointCloudSocket, pointCloudClient)

        positionRate = 60
        positionSocket = None
        positionClient = None
        positionPublisher(positionRate, positionSocket, positionClient)

        statusRate = 60
        statusSocket = None
        statusClient = None
        statusPublisher(statusRate, statusSocket, statusClient)

    except KeyboardInterrupt:
        pass

    finally:
        print("Shutting down")
        if imageSocket is not None:
            imageSocket.close()
        if imageClient is not None:
            imageClient.close()
        if pointCloudSocket is not None:
            pointCloudSocket.close()
        if pointCloudClient is not None:
            pointCloudClient.close()
        if positionSocket is not None:
            positionSocket.close()
        if positionClient is not None:
            positionClient.close()
        if statusSocket is not None:
            statusSocket.close()
        if statusClient is not None:
            statusClient.close()
        rclpy.shutdown()