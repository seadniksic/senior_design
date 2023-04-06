#include <ros/ros.h>
#include "transmitSubscriber.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roverCommandsNetworkNode");
    TransmitSubscriber newNode(WIFI_ROVER_COMMANDS_PORT, HOST_IP, ROVER_COMMANDS_BUFFER_SIZE, "roverCommandsNetworkNode", COMMANDS_DEBUG, "roverCommandsNetworkNodeSubscriber", ROVER_COMMANDS_PRIORITY);
    return 0;
}
