import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8MultiArray
import pickle

pubby = rospy.Publisher("cameraPositionNetworkNodeSubsriber", UInt8MultiArray, queue_size=10)

def odom_cb(msg):
    p = msg.pose.pose
    sendData = pickle.dumps(p)
    print(len(sendData))
    newMsg = UInt8MultiArray()
    newMsg.data = sendData
    pubby.publish(newMsg)
    pose_pub.publish(p)


rospy.init_node("poseSub")
odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, odom_cb, queue_size=100)
pose_pub = rospy.Publisher('/rover/fused_pose', Pose, queue_size=100)
rospy.spin()