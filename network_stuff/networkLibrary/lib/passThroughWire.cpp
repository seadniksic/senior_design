#ifndef PASSTHROUGHWIRE_CPP
#define PASSTHROUGHWIRE_CPP

#include "passThroughWire.h"

PassThroughWire::PassThroughWire(const int receivePort, const int transmitPort, const char *transmitIP, const int bufferSize, std::string name)
{
    this->server = std::make_unique<ReceiveData>(receivePort, name);
    this->client = std::make_unique<TransmitData>(transmitIP, transmitPort, name);
    this->buffer = std::make_unique<char[]>(bufferSize);
    this->bufferSize = bufferSize;
}

void PassThroughWire::update()
{
    int returnValue = 0;
    while(returnValue == 0)
        returnValue = this->server->getData(static_cast<void*>(this->buffer.get()), this->bufferSize);
    client->sendPayload(static_cast<void*>(this->buffer.get()), returnValue);
}

#endif