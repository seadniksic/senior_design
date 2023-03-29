import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import cv2, socket
import struct

def merge_rgb_fields(cloud_arr):
    '''Takes an array with named np.uint8 fields 'r', 'g', and 'b', and returns an array in
    which they have been merged into a single np.float32 'rgb' field. The first byte of this
    field is the 'r' uint8, the second is the 'g', uint8, and the third is the 'b' uint8.
    This is the way that pcl likes to handle RGB colors for some reason.
    '''
    r = np.asarray(cloud_arr[:, :, 0], dtype=np.uint32)
    g = np.asarray(cloud_arr[:, :, 1], dtype=np.uint32)
    b = np.asarray(cloud_arr[:, :, 2], dtype=np.uint32)    
    rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)

    # # not sure if there is a better way to do this. i'm changing the type of the array
    # # from uint32 to float32, but i don't want any conversion to take place -jdb
    rgb_arr.dtype = np.float32

    # create a new array, without r, g, and b, but with rgb float32 field
    # new_dtype = []
    # for field_name in ['r', 'g', 'b']:
    #     field_type, field_offset = cloud_arr.dtype.fields[field_name]
    #     field_type = 
    #     if field_name not in ('r', 'g', 'b'):
    #         new_dtype.append((field_name, field_type))
    # new_dtype.append(('rgb', np.float32))
    # new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)    

    # # fill in the new array
    # for field_name in new_cloud_arr.dtype.names:
    #     if field_name == 'rgb':
    #         new_cloud_arr[field_name] = rgb_arr
    #     else:
    #         new_cloud_arr[field_name] = cloud_arr[field_name]
        
    return rgb_arr

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
        xyzSize = self.client.recv(8)
        xyzSize = int.from_bytes(xyzSize, 'little')

        msg = self.client.recv(xyzSize)
        while len(msg) < xyzSize:
            msg += self.client.recv(xyzSize - len(msg))
        
        rgbSize = self.client.recv(8)
        rgbSize = int.from_bytes(rgbSize, 'little')
        
        while(len(msg) < xyzSize + rgbSize):
            msg += self.client.recv(xyzSize + rgbSize - len(msg))

        buffer = np.frombuffer(msg, np.uint8)
        xyzArray = cv2.imdecode(buffer[:xyzSize], cv2.IMREAD_COLOR)
        rgbArray = cv2.imdecode(buffer[xyzSize:], cv2.IMREAD_COLOR)
        xyzArray = xyzArray.astype(np.float32)
        rgbSingleField = merge_rgb_fields(rgbArray)
        # pointCloudArray = np.append(xyzArray, rgbSingleField)
        # print(xyzArray.dtype)
        # print(rgbSingleField.dtype)

        combinedArray = rgbSingleField.tobytes()
        print(len(combinedArray))

        pointCloudMsg = PointCloud2()
        pointCloudMsg.header.stamp = self.get_clock().now().to_msg()
        pointCloudMsg.header.frame_id = "map"
        pointCloudMsg.height = 480
        pointCloudMsg.width = 640
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        pointCloudMsg.fields = fields
        pointCloudMsg.is_bigendian = False
        pointCloudMsg.point_step = 16
        pointCloudMsg.row_step = 20480
        pointCloudMsg.is_dense = False
        pointCloudMsg.data = xyzArray.tobytes() + rgbSingleField.tobytes()
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