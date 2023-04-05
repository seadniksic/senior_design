#!/usr/bin/env python

import rospy, pickle, socket, zstd, select
from rtabmap_ros.msg import MapData

class MinimalPublisher:

    def __init__(self):
        rospy.init_node('mapDataPublisher')
        self.publisher_ = rospy.Publisher('mapDataStream', MapData, queue_size=10)
        timer_period = rospy.Duration.from_sec(0.016666)  # seconds
        self.timer = rospy.Timer(timer_period, self.timer_callback)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "127.0.0.1"
        port = 8090

        self.sock.bind((host, port))
        self.sock.listen(1)
        self.client = None


    def timer_callback(self, event):
        if self.client is None:
            readSocket, writeSocket, errorSocket = select.select([self.sock], [], [], 0)
            if len(readSocket) == 0:
                return
            self.client, addr = self.sock.accept()
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

        newMap = zstd.decompress(msg)
        newMap = pickle.loads(newMap)
        print("published!")
        self.publisher_.publish(newMap)


def main(args=None):
    minimal_publisher = MinimalPublisher()

    rospy.spin()


if __name__ == '__main__':
    main()
