#ifndef PASSTHROUGHWIRE_CPP
#define PASSTHROUGHWIRE_CPP

#include "passThroughWire.h"
/*
This code defines the implementation of a PassThroughWire class. The class constructor takes in several arguments, including receive and transmit ports, transmit IP, buffer size, and name.

The constructor initializes the class's member variables, including creating instances of ReceiveData and TransmitData classes, which are used for receiving and transmitting data respectively. It also creates a unique pointer to a char array with the size of bufferSize.

The update() function is the main method of the PassThroughWire class. It continuously receives data from the server using the getData() method of the ReceiveData class until it receives a non-zero return value. It then sends the received payload to the client using the sendPayload() method of the TransmitData class.

The code is also wrapped in an include guard, which prevents the header file from being included more than once in a translation unit.

Overall, this code defines a class that can be used to pass data through a network connection using receive and transmit ports, as well as a buffer for storing data.
*/

PassThroughWire::PassThroughWire(const int receivePort, const int transmitPort, const char *transmitIP, const int bufferSize, std::string name, int debugMode)
{
    this->server = std::make_unique<ReceiveData>(receivePort, name);
    this->client = std::make_unique<TransmitData>(transmitIP, transmitPort, name, IMAGE_FEED_PRIORITY);
    this->buffer = std::make_unique<char[]>(bufferSize);
    this->bufferSize = bufferSize;
    this->name = name;
    debug = debugMode;
}

void PassThroughWire::update()
{
    int returnValue = 0;
    while(returnValue == 0)
        returnValue = this->server->getData(static_cast<void*>(this->buffer.get()), this->bufferSize);
    if(debug)
        std::cout << name << " has received " << returnValue << " bytes..." << std::endl;
    client->sendPayload(static_cast<void*>(this->buffer.get()), returnValue);
    if(debug)
        std::cout << name << " has sent " << returnValue << " bytes..." << std::endl;
}

#endif
