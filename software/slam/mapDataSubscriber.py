import rospy, pickle, zstd
from rtabmap_ros.msg import MapData
from std_msgs.msg import UInt8MultiArray

pubby = rospy.Publisher("mapDataNetworkNodeSubscriber", UInt8MultiArray, queue_size=10)

def map_data_callback(msg):
    sendData = pickle.dumps(msg)

    sendData = zstd.compress(sendData)
    newMsg = UInt8MultiArray()
    newMsg.data = sendData
    pubby.publish(newMsg)

rospy.init_node("mapSub")
rospy.Subscriber("/rtabmap/mapData", MapData, map_data_callback)
rospy.spin()