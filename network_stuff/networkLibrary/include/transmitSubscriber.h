#ifndef TRANSMITSUBSCRIBER_H
#define TRANSMITSUBSCRIBER_H

#include <memory>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <string>
#include "transmitData.h"
#include "common.h"

class TransmitSubscriber{ 
    private:
        std::unique_ptr<TransmitData> client;
        std::unique_ptr<char[]> buffer;
        uint64_t bufferSize;
        int debug;
        std::string name;
        ros::NodeHandle n;
        ros::Subscriber transmitDataPublisher;
        void start(const std_msgs::UInt8MultiArray::ConstPtr& byte_array_msg);
    public:
        TransmitSubscriber(const int transmitPort, const char *transmitIP, const int bufferSize, std::string name, int debugMode, std::string subName, const uint8_t priority);
};

#endif