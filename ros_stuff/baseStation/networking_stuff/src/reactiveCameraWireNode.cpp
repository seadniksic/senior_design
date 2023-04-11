#include <ros/ros.h>
#include "receivePublisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactiveCameraNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    ReceivePublisher newPublisher(WIFI_REACTIVE_CAMERA_PORT, IMAGE_BUFFER_SIZE, "reactiveCameraNetworkNodePublisher", REACTIVE_CAMERA_DEBUG);
    return 0;
}