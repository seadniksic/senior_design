#include <ros/ros.h>
#include "receivePublisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roverStatusNetworkNode");
    ReceivePublisher newPublisher(WIFI_ROVER_STATUS_PORT, ROVER_STATUS_BUFFER_SIZE, "roverStatusNetworkNodePublisher", STATUS_DEBUG);
    return 0;
}
