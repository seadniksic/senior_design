#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapDataNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    PassThroughWire connection(BASE_STATION_COMMANDS_PORT, WIFI_ROVER_COMMANDS_PORT, HOST_IP, ROVER_COMMANDS_BUFFER_SIZE, "Rover Commands Wire", COMMANDS_DEBUG);

    while (ros::ok())
    {
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
