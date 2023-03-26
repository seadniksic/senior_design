#!/usr/bin/env python

import rclpy
from imageFeedPublisherNode import ImagePublisher
from pointCloudPublisherNode import PointCloudPublisher
from cameraPositionPublisher import CameraPositionPublisher
from roverStatusPublisher import RoverStatusPublisher

if __name__ == '__main__':
    try:
        rclpy.init()

        nodeList = []
        nodeList.append(ImagePublisher())
        nodeList.append(PointCloudPublisher())
        nodeList.append(CameraPositionPublisher())
        nodeList.append(RoverStatusPublisher())

        rclpy.spin(nodeList)
        
        for node in nodeList:
            node.shutdownNode()
            node.destroy_node()
        
        rclpy.shutdown()
    
    except KeyboardInterrupt:
        pass

    finally:
        for node in nodeList:
            node.shutdownNode()
