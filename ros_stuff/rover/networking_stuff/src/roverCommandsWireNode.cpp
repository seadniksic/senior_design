#include <ros/ros.h>
#include "receivePublisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roverCommandsNetworkNode");

    ReceivePublisher connection(WIFI_ROVER_COMMANDS_PORT, ROVER_COMMANDS_BUFFER_SIZE, "roverCommandsNetworkNodePublisher", COMMANDS_DEBUG);
    return 0;
}