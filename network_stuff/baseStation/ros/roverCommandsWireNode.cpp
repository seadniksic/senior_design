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
    PassThroughWire connection(BASE_STATION_COMMANDS_PORT, WIFI_ROVER_COMMANDS_PORT, HOST_IP, ROVER_COMMANDS_BUFFER_SIZE, "Rover Commands Wire", COMMANDS_DEBUG);

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
