#ifndef BASESTATIONNETWORKING_CPP
#define BASESTATIONNETWORKING_CPP

#include "baseStationNetworking.h"
#include <iostream>

ReceiveData *imageServer;
ReceiveData *slamServer;
ReceiveData *statusServer;
TransmitData *commandsClient;

TransmitData *imageClient;
TransmitData *slamClient;
TransmitData *statusClient;
ReceiveData *commandsServer;

char *cameraBuffer, *slamBuffer, *statusBuffer, *commandsBuffer;

void initializeNetwork()
{
    imageServer = new ReceiveData(WIFI_IMAGE_PORT);
    slamServer = new ReceiveData(WIFI_SLAM_PORT);
    statusServer = new ReceiveData(WIFI_ROVER_STATUS_PORT);
    commandsClient = new TransmitData(HOST_IP, WIFI_ROVER_COMMANDS_PORT);

    imageClient = new TransmitData(LOCAL_IP, BASE_STATION_IMAGE_PORT);
    slamClient = new TransmitData(LOCAL_IP, BASE_STATION_SLAM_PORT);
    statusClient = new TransmitData(LOCAL_IP, BASE_STATION_STATUS_PORT);
    commandsServer = new ReceiveData(BASE_STATION_COMMANDS_PORT);

    cameraBuffer = new char[IMAGE_HEIGHT * IMAGE_WIDTH * 3];
    slamBuffer = new char[IMAGE_HEIGHT * IMAGE_WIDTH];
    statusBuffer = new char[200];
    commandsBuffer = new char[200];
}

void shutdownNetwork()
{
    delete imageServer;
    delete slamServer;
    delete statusServer;
    delete commandsClient;

    delete imageClient;
    delete slamClient;
    delete statusClient;
    delete commandsServer;

    delete [] cameraBuffer;
    delete [] slamBuffer;
    delete [] statusBuffer;
    delete [] commandsBuffer;
}

void getCameraData()
{
    size_t bufferSize = IMAGE_HEIGHT * IMAGE_WIDTH * 3;
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = imageServer->getData(cameraBuffer, bufferSize);
    imageClient->sendPayload(cameraBuffer, returnVal);
}

void getSlamData()
{
    size_t bufferSize = IMAGE_HEIGHT * IMAGE_WIDTH;
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = slamServer->getData(slamBuffer, bufferSize);
    slamClient->sendPayload(slamBuffer, returnVal);
}

void getRoverStatus()
{
    size_t bufferSize = sizeof(char) * 200;
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = statusServer->getData(statusBuffer, bufferSize);
    statusClient->sendPayload(statusBuffer, returnVal); 
}

void sendRoverCommands()
{
    size_t bufferSize = sizeof(char) * 200;

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