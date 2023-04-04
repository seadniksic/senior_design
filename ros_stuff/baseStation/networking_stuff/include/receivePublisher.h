#ifndef RECEIVEPUBLISHER_H
#define RECEIVEPUBLISHER_H

#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include "receiveData.h"
#include "common.h"

class ReceivePublisher{
    private:
        std::unique_ptr<ReceiveData> server;
        std::unique_ptr<char[]> buffer;
        uint64_t bufferSize;
        int debug;
        std::string name;
        ros::NodeHandle n;
        ros::Publisher receiveDataPublisher;
        ros::Timer timer;
        static void start(const ros::TimerEvent& event);
    public:
        ReceivePublisher(const int receivePort, const int bufferSize, std::string name, int debugMode);
};


#endif