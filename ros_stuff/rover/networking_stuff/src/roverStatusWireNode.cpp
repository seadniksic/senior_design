#include <ros/ros.h>
#include "transmitSubscriber.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roverStatusNetworkNode");
    TransmitSubscriber newNode(WIFI_ROVER_STATUS_PORT, CLIENT_IP, ROVER_STATUS_BUFFER_SIZE, "roverStatusNetworkNode", STATUS_DEBUG, "roverStatusNetworkNodeSubsriber", ROVER_STATUS_PRIORITY);
    return 0;
}