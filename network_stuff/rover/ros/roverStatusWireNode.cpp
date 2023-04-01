#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraFeedNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    // your code here
    PassThroughWire connection(JETSON_STATUS_PORT, WIFI_ROVER_STATUS_PORT, CLIENT_IP, ROVER_STATUS_BUFFER_SIZE, "Rover Status Wire", STATUS_DEBUG);

    while (ros::ok())
    {
        // your code here
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}