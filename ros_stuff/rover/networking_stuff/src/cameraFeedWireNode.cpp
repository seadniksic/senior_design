#include <ros/ros.h>
#include "transmitSubscriber.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraFeedNetworkNode");
    TransmitSubscriber newNode(WIFI_IMAGE_PORT, CLIENT_IP, IMAGE_BUFFER_SIZE, "cameraFeedNetworkNode", IMAGE_DEBUG, "cameraFeedNetworkNodeSubscriber", IMAGE_FEED_PRIORITY);
    return 0;
}