#include <ros/ros.h>
#include "transmitSubscriber.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraPositionNetworkNode");
    TransmitSubscriber newNode(WIFI_CAMERA_LOCATION_PORT, CLIENT_IP, CAMERA_POSITION_BUFFER_SIZE, "cameraPositionNetworkNode", POISITON_DEBUG, "cameraPositionNetworkNodeSubsriber", CAMERA_POSITION_PRIORITY);
    return 0;
}