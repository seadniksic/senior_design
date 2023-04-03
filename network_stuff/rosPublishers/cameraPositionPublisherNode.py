#!/usr/bin/env python

import rospy, socket, select, pickle
from geometry_msgs.msg import Point

class CameraPositionPublisher:
    def __init__(self):
        rospy.init_node('cameraPositionPublisher')
        self.publisher_ = rospy.Publisher('cameraPositionStream', Point, queue_size=10)
        timer_period = rospy.Duration.from_sec(0.016666)  # seconds
        self.timer = rospy.Timer(timer_period, self.timer_callback)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(("127.0.0.1", 8094))
        self.socket.listen(1)

        self.client = None
    
    def timer_callback(self, event):
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
            self.publisher_.publish(pointData)

        except Exception as e:
            print("Image Publisher ran into " + str(e))
            print("Attempting to reconnect...")
            
            if self.client is not None:
                self.client.close()
            self.client = None
    
    def shutdownNode(self):
        if self.socket is not None:
            self.socket.close()
        if self.client is not None:
            self.client.close()

if __name__ == '__main__':
    try:
        cameraPositionNode = CameraPositionPublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass

    finally:
        cameraPositionNode.shutdownNode()