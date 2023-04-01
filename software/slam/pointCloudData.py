import rospy
import lz4.frame
import socket
from rtabmap_ros.msg import MapData
import socket


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8085
sock.connect((host, port))

# Define a callback function to receive the PointCloud2 message
def callback(msg):
    
    sendData = lz4.frame.compress(msg.data)
    # print(len(sendData))
    size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    sock.sendall(size)
    sock.sendall(sendData)


# Initialize the ROS node
rospy.init_node('point_cloud_converter')

# Subscribe to the PointCloud2 topic
rospy.Subscriber('/camera/depth_registered/points', MapData, callback)

# Spin the ROS node to receive messages
rospy.spin()
