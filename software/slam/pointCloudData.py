import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8085
sock.connect((host, port))

# Define a callback function to receive the PointCloud2 message
def callback(point_cloud_msg):
    # Define the fields to extract from the PointCloud2 message
    fields = [
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('r', np.uint8),
        ('g', np.uint8),
        ('b', np.uint8)
    ]

    # Convert the PointCloud2 message to a generator of tuples
    point_cloud_generator = pc2.read_points(point_cloud_msg, field_names=fields)

    # Convert the generator to a list of tuples
    points_list = list(point_cloud_generator)

    # Convert the list of tuples to a NumPy array
    points_array = np.array(points_list)

    __, sendData = cv2.imencode('.jpg', points_array, [int(cv2.IMWRITE_JPEG_QUALITY), 50])

    sendData = sendData.tobytes()
    size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    sock.sendall(size)
    sock.sendall(sendData)


# Initialize the ROS node
rospy.init_node('point_cloud_converter')

# Subscribe to the PointCloud2 topic
rospy.Subscriber('/my/point_cloud_topic', pc2.PointCloud2, callback)

# Spin the ROS node to receive messages
rospy.spin()
