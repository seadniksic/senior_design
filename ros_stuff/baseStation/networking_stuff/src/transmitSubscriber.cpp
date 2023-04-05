#ifndef TRANSMITLISTENER_CPP
#define TRANSMITLISTENER_CPP

#include "transmitSubscriber.h"

TransmitSubscriber::TransmitSubscriber(const int transmitPort, const char *transmitIP, const int bufferSize, std::string name, int debugMode, std::string subName)
{
    this->client = std::make_unique<TransmitData>(transmitIP, transmitPort, name);
    this->buffer = std::make_unique<char[]>(bufferSize);
    this->bufferSize = bufferSize;
    this->name = name;
    this->debug = debugMode;

    transmitDataPublisher = n.subscribe(subName, 1000, this->start);

    ros::spin();
}

TransmitSubscriber::start(const std_msgs::UInt8MultiArray::ConstPtr& byte_array_msg)
{
    this->client->sendPayload(static_cast<void*>(byte_array_msg->data) , byte_array_msg->data);
}

#endif