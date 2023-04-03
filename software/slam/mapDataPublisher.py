import rclpy, pickle, socket, zstd, select
from rclpy.node import Node
from rtabmap_ros.msg import MapData
from rosMapToRos2Map import Ros1MapDataBridge

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

        newMap = zstd.decompress(msg)
        newMap = pickle.loads(newMap)

        self.publisher_.publish(newMap.getMapData())


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