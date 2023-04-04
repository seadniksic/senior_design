#include <ros/ros.h>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraPositionNetworkNode");
    ros::NodeHandle newNode;
    ros::Rate rate(60);

    PassThroughWire connection(WIFI_CAMERA_LOCATION_PORT, BASE_STATION_CAMERA_LOCATION_PORT, LOCAL_IP, CAMERA_POSITION_BUFFER_SIZE, "Camera Position Wire", POISITON_DEBUG);

    while (ros::ok())
    {
        connection.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
