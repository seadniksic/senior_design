import rclpy
import pclpy
from rclpy.node import Node
import pickle, zstd
from rtabmap_ros.msg import MapData
import socket, io
from pclpy import pcl


class RenameUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        renamed_module = module
        if module == "rtabmap_ros.msg._MapData":
            renamed_module = "rtabmap_ros.msg._map_data"
        if module == "std_msgs.msg._Header":
            renamed_module = "std_msgs.msg._header"
        if module == "genpy.rostime":
            renamed_module = "rclpy.time"
        if module == "rtabmap_ros.msg._NodeData":
            renamed_module = "rtabmap_ros.msg._node_data"
        if module == "geometry_msgs.msg._Pose":
            renamed_module = "geometry_msgs.msg._pose"
        if module == "geometry_msgs.msg._Point":
            renamed_module = "geometry_msgs.msg._point"

        return super(RenameUnpickler, self).find_class(renamed_module, name)


def renamed_load(file_obj):
    currVal = RenameUnpickler(file_obj)
    print(type(currVal))
    return currVal.load()


def renamed_loads(pickled_bytes):
    file_obj = io.BytesIO(pickled_bytes)
    return renamed_load(file_obj)

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

        compression = pcl.OctreePointCloudCompression.PointXYZRGB()
        compression.setCompressionOptions(8, True)

        # Decompress the compressed data
        decompressed_cloud = pclpy.pcl.PointCloud.PointXYZRGB()
        compression.decompressPointCloud(msg, decompressed_cloud)

        # Convert the decompressed PCL PointCloud<PointXYZRGB> object back to a PointCloud2 message
        decompressed_msg = pclpy.pcl.toROSMsg(decompressed_cloud)
        # decompressed_msg.header = 
        # decompressed_msg.fields = fields
        decompressed_msg.is_dense = True

        self.publisher_.publish(decompressed_msg)


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