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
    PassThroughWire connection(WIFI_ROVER_STATUS_PORT, BASE_STATION_STATUS_PORT, LOCAL_IP, ROVER_STATUS_BUFFER_SIZE, "Rover Status Wire", STATUS_DEBUG);

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
