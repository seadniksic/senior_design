#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraFeedNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    // your code here
    PassThroughWire connection(JETSON_POINT_CLOUD_PORT, WIFI_POINT_CLOUD_PORT, CLIENT_IP, POINT_CLOUD_BUFFER_SIZE, "Point Cloud Wire", POINT_CLOUD_DEBUG);

    while (ros::ok())
    {
        // your code here
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}