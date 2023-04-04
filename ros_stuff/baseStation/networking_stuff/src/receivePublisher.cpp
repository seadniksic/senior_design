#ifndef RECEIVEPUBLISHER_CPP
#define RECEIVEPUBLISHER_CPP

#include "receivePublisher.h"

ReceivePublisher::ReceivePublisher(const int receivePort, const int bufferSize, std::string name, int debugMode)
{

    this->server = std::make_unique<ReceiveData>(receivePort, name);
    this->buffer = std::make_unique<char[]>(bufferSize);
    this->bufferSize = bufferSize;
    this->name = name;
    this->debug = debugMode;

    this->receiveDataPublisher = n.advertise<std_msgs::UInt8MultiArray>(name, 10);

    this->timer = n.createTimer(ros::Duration(0.016666), &ReceivePublisher::start, this);

    ros::spin();
}

void ReceivePublisher::start(const ros::TimerEvent& event)
{
    std_msgs::UInt8MultiArray newMsg;

    int returnSize = this->server->getData(static_cast<void*>(this->buffer.get()), this->bufferSize);
    if(returnSize > 0)
    {
        if(debug)
            std::cout << name << " has wirelessly received " << returnSize << " bytes...\n" << std::endl;
        
        std::memcpy(neMsg.data(), buffer, returnSize);
        receiveDataPublisher.publish(newMsg);
    }
}

#endif