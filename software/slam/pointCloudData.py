import pclpy
from pclpy import pcl
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import socket, rospy

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8085
sock.connect((host, port))

def map_data_callback(data):
    # Convert the PointCloud2 message to a PCL PointCloud<PointXYZRGB> object
    pcl_cloud = pclpy.pcl.PointCloud.PointXYZRGB()
    pclpy.pcl.PointCloud.from_msg(data, pcl_cloud)

    # Create an OctreePointCloudCompression object and set the desired compression options
    compression = pcl.OctreePointCloudCompression.PointXYZRGB()
    compression.setCompressionOptions(8, True)

    # Compress the point cloud and retrieve the compressed data
    compressed_data = pclpy.pcl.CompressedData()
    compression.compressPointCloud(pcl_cloud, compressed_data)
    compressed_data_str = compressed_data.getData()

    size = int(len(compressed_data_str)).to_bytes(8, byteorder="little", signed=False)

    sock.sendall(size)
    sock.sendall(compressed_data_str)

rospy.init_node("pc_sub")
rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, map_data_callback)
rospy.spin()
sock.close()

