import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2, socket

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'pointCloudStream', 10)
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
        msg = self.client.recv(1400)
        while len(msg) < size:
            msg += self.client.recv(min(1400, size - len(msg)))
        

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
        self.publisher_.publish(pointCloudMsg)


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