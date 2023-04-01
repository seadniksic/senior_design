import rclpy
from rclpy.node import Node
import lz4.frame
from rtabmap_ros.msg import MapData
import socket


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MapData, 'mapDataStream', 10)
        timer_period = 0.016666  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "127.0.0.1"
        port = 8090

        self.sock.bind((host, port))
        self.sock.listen(1)
        self.client, addr = self.sock.accept()
        self.readSockets = [self.client]


    def timer_callback(self):

        size = self.client.recv(8)
        size = int.from_bytes(size, 'little')
        if size <= 0:
            return

        msg = self.client.recv(size)
        while len(msg) < size:
            msg += self.client.recv(size - len(msg))

        newMapData = lz4.frame.decompress(msg)
        mapData = MapData()
        mapData.header = None #Need to implemenmt
        mapData.id = None #need to implement
        mapData.data = newMapData

        self.publisher_.publish(mapData)


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