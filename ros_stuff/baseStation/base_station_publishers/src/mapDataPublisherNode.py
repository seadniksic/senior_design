#!/usr/bin/env python

import rospy, pickle, zstd
from rtabmap_ros.msg import MapData
from std_msgs.msg import UInt8MultiArray

class MapDataPublisher:
    def __init__(self):
        rospy.init_node('mapDataPublisher')
        self.publisher_ = rospy.Publisher('mapDataStream', MapData, queue_size=10)
        self.subscriber_ = rospy.Subscriber('mapDataNetworkNodePublisher', UInt8MultiArray, self.callback)
        rospy.spin()

    def callback(self, msg):
        try:
            newMap = zstd.decompress(msg.data)
            newMap = pickle.loads(newMap)
            self.publisher_.publish(newMap)

        except Exception as e:
            print("Image Publisher ran into: " + str(e))

if __name__ == '__main__':
    try:
        mapNode = MapDataPublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass
