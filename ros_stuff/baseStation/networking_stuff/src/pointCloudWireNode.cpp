#include <ros/ros.h>
#include "receivePublisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapDataNetworkNode");
    ReceivePublisher newPublisher(WIFI_POINT_CLOUD_PORT, POINT_CLOUD_BUFFER_SIZE, "mapDataNetworkNodePublisher", POINT_CLOUD_DEBUG);
    return 0;
}
