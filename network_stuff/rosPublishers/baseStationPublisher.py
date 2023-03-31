#!/usr/bin/env python

import launch
import launch_ros.actions
from imageFeedPublisherNode import ImagePublisher
from pointCloudPublisherNode import PointCloudPublisher
from cameraPositionPublisher import CameraPositionPublisher
from roverStatusPublisher import RoverStatusPublisher

if __name__ == '__main__':
    try:
        rclpy.init()

        executor = rclpy.executors.MultiThreadedExecutor()
        nodeList = []
        nodeList.append(ImagePublisher())
        executor.add_node(nodeList[0])
        nodeList.append(PointCloudPublisher())
        executor.add_node(nodeList[1])
        nodeList.append(CameraPositionPublisher())
        executor.add_node(nodeList[2])
        nodeList.append(RoverStatusPublisher())
        executor.add_node(nodeList[3])

        rclpy.spin(executor)
        
        for node in nodeList:
            node.shutdownNode()
            node.destroy_node()
        
        rclpy.shutdown()
    
    except KeyboardInterrupt:
        pass

    finally:
        for node in nodeList:
            node.shutdownNode()

