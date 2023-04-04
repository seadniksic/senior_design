#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraFeedNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    PassThroughWire connection(WIFI_IMAGE_PORT, BASE_STATION_IMAGE_PORT, LOCAL_IP, IMAGE_BUFFER_SIZE, "Image Feed Wire", IMAGE_DEBUG);
 
    while (ros::ok())
    {
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
