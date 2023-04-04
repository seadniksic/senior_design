#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapDataNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);


    // your code here
    PassThroughWire connection(WIFI_POINT_CLOUD_PORT, BASE_STATION_POINT_CLOUD_PORT, LOCAL_IP, POINT_CLOUD_BUFFER_SIZE, "Point Cloud Wire", POINT_CLOUD_DEBUG);

    while (ros::ok())
    {
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
