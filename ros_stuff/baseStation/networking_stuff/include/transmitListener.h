#ifndef TRANSMITLISTENER_H
#define TRANSMITLISTENER_H

#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include "transmitData.h"
#include "common.h"

class TransmitListener{
    private:
        std::unique_ptr<TransmitData> client;
        std::unique_ptr<char[]> buffer;
        uint64_t bufferSize;
        int debug;
        std::string name;
        ros::NodeHandle n;
        ros::Subscriber transmitDataPublisher;
        void start(const std_msgs::String::ConstPtr& msg);
    public:
        TransmitListener(const int transmitPort, const char *transmitIP, const int bufferSize, std::string name, int debugMode, std::string subName);
};

#endif