#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "passThroughWire.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cameraFeedNetworkNode");
    rclcpp::Rate rate(60);

    // your code here
    PassThroughWire connection(WIFI_POINT_CLOUD_PORT, BASE_STATION_POINT_CLOUD_PORT, LOCAL_IP, POINT_CLOUD_BUFFER_SIZE, "Point Cloud Wire", POINT_CLOUD_DEBUG);

    while (rclcpp::ok())
    {
        // your code here
        connection.update();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
