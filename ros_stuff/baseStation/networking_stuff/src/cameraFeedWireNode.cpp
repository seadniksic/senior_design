#include <ros/ros.h>
#include "receivePublisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraFeedNetworkNode");
    ReceivePublisher newPublisher(WIFI_IMAGE_PORT, IMAGE_BUFFER_SIZE, "cameraFeedNetworkNodePublisher", IMAGE_DEBUG);
    newPublisher.start();
    return 0;
}
