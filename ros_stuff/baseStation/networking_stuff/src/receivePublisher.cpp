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

    this->receiveDataPublisher = n.advertise<std_msgs::String>(name, 10);

    timer = node_handle.createTimer(ros::Duration(0.016666), this->start);

    ros::spin();
}

void ReceivePublisher::start(const ros::TimerEvent& event)
{
    std_msgs::String newMsg;
    std::string newString;

    int returnSize = this->server->getData(static_cast<void*>(this->buffer.get()), this->bufferSize);
    if(returnSize > 0)
    {
        if(debug)
            std::cout << name << " has wirelessly received " << returnSize << " bytes...\n" << std::endl;
        newString.append(buffer.get(), returnSize);
        newMsg.data = newString;

        receiveDataPublisher.publish(newMsg);
    }
}

#endif