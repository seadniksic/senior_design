import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import socket
import ros_numpy as rnp
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8085
sock.connect((host, port))

# Define a callback function to receive the PointCloud2 message
def callback(point_cloud_msg):
    # Define the fields to extract from the PointCloud2 message
    # print(len(point_cloud_msg.data['x']))
    points_array = rnp.point_cloud2.pointcloud2_to_xyz_array(point_cloud_msg, remove_nans=False)
    # rgb = ros_numpy.point_cloud2.pointcloud2_to_rgb_array(point_cloud_msg)
    # # Convert the PointCloud2 message to a generator of tuples
    # point_cloud_generator = pc2.read_points(point_cloud_msg, field_names=["x","y","z","rgb"], skip_nans=True)
    # # Convert the generator to a list of tuples
    # points_list = list(point_cloud_generator)
    # # Convert the list of tuples to a NumPy array
    # points_array = np.array(points_list)
    # print(points_array.shape)
    #points_array = xyz + rgb
    #print(len(point_cloud_msg.data))
    points = rnp.point_cloud2.pointcloud2_to_array(point_cloud_msg)
    cloud_array = rnp.point_cloud2.split_rgb_field(points)
    # rgb = rgb["r"] + rgb['g'] + rgb['b']
    xyz = np.zeros(cloud_array.shape + (3,), dtype=np.uint8)
    rgb = np.zeros(cloud_array.shape + (3,), dtype=np.uint8)
    xyz[...,0] = cloud_array['x']
    xyz[...,1] = cloud_array['y']
    xyz[...,2] = cloud_array['z']

    rgb[...,0] = cloud_array['r']
    rgb[...,1] = cloud_array['g']
    rgb[...,2] = cloud_array['b']

    #print(points_array.shape)
    #new_array = rnp.point_cloud2.array_to_pointcloud2(points_array)
    # print(len(new_array.data))
    # print(points_array.dtype)
    _, sendData = cv2.imencode('.jpg', xyz, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    __, sendData2 = cv2.imencode('.jpg', rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    # sendData = points_array
    #print(sendData.dtype)
    sendData = sendData.tobytes()
    sendData = int(len(sendData)).to_bytes(8, byteorder="little", signed=False) + sendData
    sendData2 = sendData2.tobytes()
    sendData2 = int(len(sendData2)).to_bytes(8, byteorder="little", signed=False) + sendData2
    # print(len(sendData), len(sendData2))
    sendData = sendData + sendData2
    
    
    # print(len(sendData))
    size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    sock.sendall(size)
    sock.sendall(sendData)


# Initialize the ROS node
rospy.init_node('point_cloud_converter')

# Subscribe to the PointCloud2 topic
rospy.Subscriber('/camera/depth_registered/points', pc2.PointCloud2, callback)

# Spin the ROS node to receive messages
rospy.spin()
