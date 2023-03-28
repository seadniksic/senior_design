#ifndef BASESTATIONGUINETWORKING_CPP
#define BASESTATIONGUINETWORKING_CPP

#include "baseStationGUINetworking.h"
#include <iostream>

ReceiveData *imageServer;
ReceiveData *slamServer;
ReceiveData *statusServer;
TransmitData *commandsClient;
ReceiveData *commandsServer;

char *commandsBuffer;

void initializeNetwork()
{
    imageServer = new ReceiveData(WIFI_IMAGE_PORT);
    slamServer = new ReceiveData(WIFI_SLAM_PORT);
    statusServer = new ReceiveData(WIFI_ROVER_STATUS_PORT);
    commandsClient = new TransmitData(HOST_IP, WIFI_ROVER_COMMANDS_PORT);
    commandsServer = new ReceiveData(BASE_STATION_COMMANDS_PORT);

    commandsBuffer = new char[150];
}

void shutdownNetwork()
{
    delete imageServer;
    delete slamServer;
    delete statusServer;
    delete commandsClient;

    delete [] commandsBuffer;
}

int getCameraData(image_t *buffer, size_t bufferSize)
{
    int returnVal = imageServer->getData(buffer, bufferSize);
    return returnVal; 
}

int getSlamData(slam_t *buffer, size_t bufferSize)
{
    int returnVal = slamServer->getData(buffer, bufferSize);
    return returnVal;   
}

int getRoverStatus(status_t *buffer, size_t bufferSize)
{
    int returnVal = statusServer->getData(buffer, bufferSize);
    return returnVal;   
}

void sendRoverCommands()
{
    size_t bufferSize = sizeof(char) * 150;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = commandsServer->getData(commandsBuffer, bufferSize);
    }
    std::cout << "Command byte string is: " << incomingSize << std::endl;
    for(int i = 0; i < incomingSize; i++)
        std::cout << commandsBuffer[i] << ", ";
    std::cout << std::endl;
    commandsClient->sendPayload(commandsBuffer, incomingSize);
}


#endif