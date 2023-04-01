#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraFeedNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    // your code here
    PassThroughWire connection(WIFI_ROVER_COMMANDS_PORT, JETSON_COMMANDS_PORT, LOCAL_IP, ROVER_COMMANDS_BUFFER_SIZE, "Rover Commands Wire", COMMANDS_DEBUG);

    while (ros::ok())
    {
        // your code here
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}