#include <ros/ros.h>
#include "transmitSubscriber.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapDataNetworkNode");
    TransmitSubscriber newNode(WIFI_ROVER_COMMANDS_PORT, HOST_IP, ROVER_COMMANDS_BUFFER_SIZE, "mapDataNetworkNodeSubscriber", COMMANDS_DEBUG);
    return 0;
}
