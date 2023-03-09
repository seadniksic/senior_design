#ifndef BASESTATIONNETWORKING_CPP
#define BASESTATIONNETWORKING_CPP

#include "baseStationNetworking.h"


ReceiveData<image_t> *imageServer;
ReceiveData<slam_t> *slamServer;
ReceiveData<status_t> *statusServer;
TransmitData<commands_t> *commandsClient;

void initializeNetwork()
{
    imageServer = new ReceiveData<image_t>(WIFI_IMAGE_PORT);
    slamServer = new ReceiveData<slam_t>(WIFI_SLAM_PORT);
    statusServer = new ReceiveData<status_t>(WIFI_ROVER_STATUS_PORT);
    commandsClient = new TransmitData<commands_t>(HOST_IP, WIFI_ROVER_COMMANDS_PORT);
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

void sendRoverCommands(commands_t *commands, size_t bufferSize)
{
    commandsClient->sendPayload(commands, bufferSize);
}

#endif