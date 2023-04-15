import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray
import pickle
from tf.transformations import *

pubby = rospy.Publisher("cameraPositionNetworkNodeSubsriber", UInt8MultiArray, queue_size=10)

offset = None
p = None

def odom_cb(msg):
    global p
    p = msg.pose.pose
    sendData = pickle.dumps(p)
    print(len(sendData))
    newMsg = UInt8MultiArray()
    newMsg.data = sendData
    pubby.publish(newMsg)
    pose_pub.publish(p)

def imu_cb(msg):
    global p
    if offset == None:
        offset 

rospy.init_node("poseSub")
odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, odom_cb, queue_size=100)
imu_sub = rospy.Subscriber('/rtabmap/odom', Imu, imu_cb, queue_size=100)
pose_pub = rospy.Publisher('/rover/fused_pose', Pose, queue_size=100)
rospy.spin()
