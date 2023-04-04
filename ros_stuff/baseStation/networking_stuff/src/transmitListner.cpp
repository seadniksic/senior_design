#ifndef TRANSMITLISTENER_CPP
#define TRANSMITLISTENER_CPP

#include "transmitListener.h"

TransmitListener::TransmitListener(const int transmitPort, const char *transmitIP, const int bufferSize, std::string name, int debugMode, std::string subName)
{
    ros::init(argc, argv, name);

    this->client = std::make_unique<TransmitData>(transmitIP, transmitPort, name);
    this->buffer = std::make_unique<char[]>(bufferSize);
    this->bufferSize = bufferSize;
    this->name = name;
    this->debug = debugMode;

    transmitDataPublisher = n.subscribe(subName, 1000, this->start);

    ros::spin();
}

TransmitListener::start(const std_msgs::String::ConstPtr& msg)
{
    this->client->sendPayload(static_cast<void*>(const_cast<std_msgs::String::ConstPtr&>(msg).get()) , static_cast<int>(message_string.length()));
}

#endif