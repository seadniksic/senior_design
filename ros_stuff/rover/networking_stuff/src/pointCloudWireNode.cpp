#include <ros/ros.h>
#include "transmitSubscriber.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapDataNetworkNode");
    TransmitSubscriber newNode(WIFI_POINT_CLOUD_PORT, CLIENT_IP, POINT_CLOUD_BUFFER_SIZE, "mapDataNetworkNode", POINT_CLOUD_DEBUG, "mapDataNetworkNodeSubscriber", POINT_CLOUD_PRIORITY);
    return 0;
}