#include <ros/ros.h>
#include "receivePublisher.h"

int main(int argc, char **argv)
{
    ReceivePublisher newPublisher(WIFI_IMAGE_PORT, IMAGE_BUFFER_SIZE, "cameraFeedNetworkNode", IMAGE_DEBUG);
    newPublisher.start();
    return 0;
}
