#include <ros/ros.h>
#include "receivePublisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraPositionNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    ReceivePublisher newPublisher(WIFI_CAMERA_LOCATION_PORT, CAMERA_POSITION_BUFFER_SIZE, "cameraPositionNetworkNodePublisher", POISITON_DEBUG);
    return 0;
}
