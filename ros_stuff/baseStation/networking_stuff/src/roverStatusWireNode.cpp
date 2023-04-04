#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roverStatusNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    PassThroughWire connection(WIFI_ROVER_STATUS_PORT, BASE_STATION_STATUS_PORT, LOCAL_IP, ROVER_STATUS_BUFFER_SIZE, "Rover Status Wire", STATUS_DEBUG);
    
    while (ros::ok())
    {
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
