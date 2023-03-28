import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

class MapBuilder:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('map_builder')

        # Create a publisher to publish the map
        self.map_publisher = rospy.Publisher('map', PointCloud2, queue_size=10)

        # Create a subscriber to receive robot position data
        self.pose_subscriber = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, self.pose_callback)

        # Create a subscriber to receive point cloud data
        self.point_cloud_subscriber = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.point_cloud_callback)

        # Initialize the map as an empty array
        self.map = np.zeros((100, 100, 100), dtype=np.uint8)

        # Initialize the robot's position
        self.robot_position = np.zeros(3)

    def pose_callback(self, msg):
        # Update the robot's position based on the pose data
        self.robot_position[0] = msg.pose.position.x
        self.robot_position[1] = msg.pose.position.y
        self.robot_position[2] = msg.pose.position.z

    def point_cloud_callback(self, msg):
        # Convert the point cloud message to a numpy array
        point_cloud = np.array(list(pc2.read_points(msg)))

        # Transform the point cloud to the robot's frame of reference
        # (assuming the point cloud is in the world frame)
        point_cloud[:, 0] -= self.robot_position[0]
        point_cloud[:, 1] -= self.robot_position[1]
        point_cloud[:, 2] -= self.robot_position[2]


        # Publish the updated map
        map_msg = pc2.create_cloud_xyz32(msg.header, point_cloud)
        self.map_publisher.publish(map_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    map_builder = MapBuilder()
    map_builder.run()

