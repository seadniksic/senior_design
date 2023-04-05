#!/usr/bin/env python

import rospy
from rtabmap_ros.msg import MapData
from sensor_msgs.msg import PointCloud2
pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
def map_data_callback(data):
    # This function will be called whenever a new MapData message is received
    # You can access the map data fields here and do something with them
    print(data.nodes)
    pub.publish(data.nodes.wordPts)

if __name__ == '__main__':
    rospy.init_node('map_data_subscriber', anonymous=True)

    # Create a subscriber object that will listen for MapData messages
    
    rospy.Subscriber('/rtabmap/mapData', MapData, map_data_callback)

    # Spin the node to keep it alive and processing messages
    rospy.spin()