#ifndef TRANSMITLISTENER_CPP
#define TRANSMITLISTENER_CPP

#include "transmitSubscriber.h"

TransmitSubscriber::TransmitSubscriber(const int transmitPort, const char *transmitIP, const int bufferSize, std::string name, int debugMode, std::string subName, const uint8_t priority)
{
    this->client = std::make_unique<TransmitData>(transmitIP, transmitPort, name, priority);
    this->buffer = std::make_unique<char[]>(bufferSize);
    this->bufferSize = bufferSize;
    this->name = name;
    this->debug = debugMode;

    transmitDataPublisher = n.subscribe(subName, 1000, &TransmitSubscriber::start, this);

    ros::spin();
}

void TransmitSubscriber::start(const std_msgs::UInt8MultiArray::ConstPtr& byte_array_msg)
{
    if(debug)
        std::cout << name << " will wirelessly transmit " << byte_array_msg->data.size() << " bytes...\n" << std::endl;
    this->client->sendPayload(reinterpret_cast<const void*>(byte_array_msg->data.data()), byte_array_msg->data.size());
}

#endif