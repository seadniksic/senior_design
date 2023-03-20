#ifndef BASESTATIONNETWORKING_CPP
#define BASESTATIONNETWORKING_CPP

#include "baseStationNetworking.h"
#include <iostream>

ReceiveData *imageServer;
ReceiveData *slamServer;
ReceiveData *statusServer;
TransmitData *commandsClient;
ReceiveData *commandsServer;

void initializeNetwork()
{
    imageServer = new ReceiveData(WIFI_IMAGE_PORT);
    slamServer = new ReceiveData(WIFI_SLAM_PORT);
    statusServer = new ReceiveData(WIFI_ROVER_STATUS_PORT);
    commandsClient = new TransmitData(HOST_IP, WIFI_ROVER_COMMANDS_PORT);
    commandsServer = new ReceiveData(BASE_STATION_COMMANDS_PORT);
}

void shutdownNetwork()
{
    delete imageServer;
    delete slamServer;
    delete statusServer;
    delete commandsClient;
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
    commands_t buffer;
    size_t bufferSize = sizeof(buffer);

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = commandsServer->getData(&buffer, bufferSize);
    }
    std::cout << incomingSize << std::endl;
    commandsClient->sendPayload(&buffer, incomingSize);
}


#endif