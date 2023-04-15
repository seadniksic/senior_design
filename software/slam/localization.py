import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray
import pickle
from tf.transformations import *

pubby = rospy.Publisher(
    "cameraPositionNetworkNodeSubsriber", UInt8MultiArray, queue_size=10
)

offset = None
imu_heading = None


def odom_cb(msg):
    global offset
    global imu_heading
    
    if offset is None and imu_heading is not None:
        offset = imu_heading

    p = msg.pose.pose
    if offset:
        q1 = offset
        q1[3] = -q1[3]
        p.orientation = quaternion_multiply(imu_heading, q1)

    sendData = pickle.dumps(p)

    newMsg = UInt8MultiArray()
    newMsg.data = sendData
    pubby.publish(newMsg)
    pose_pub.publish(p)


def imu_cb(msg):
    global imu_heading
    imu_heading = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    ]


rospy.init_node("poseSub")
odom_sub = rospy.Subscriber("/rtabmap/odom", Odometry, odom_cb, queue_size=100)
imu_sub = rospy.Subscriber("/rtabmap/odom", Imu, imu_cb, queue_size=100)
pose_pub = rospy.Publisher("/rover/fused_pose", Pose, queue_size=100)
rospy.spin()
