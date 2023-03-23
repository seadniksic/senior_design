#ifndef BASESTATIONNETWORKING_CPP
#define BASESTATIONNETWORKING_CPP

#include "baseStationNetworking.h"

std::unique_ptr<ReceiveData> imageServer, slamServer, statusServer, commandsServer;
std::unique_ptr<TransmitData> imageClient, slamClient, statusClient, commandsClient;
std::unique_ptr<char[]> cameraBuffer, slamBuffer, statusBuffer, commandsBuffer;

void initializeNetwork()
{
    imageServer = std::make_unique<ReceiveData>(WIFI_IMAGE_PORT);
    slamServer = std::make_unique<ReceiveData>(WIFI_SLAM_PORT);
    statusServer = std::make_unique<ReceiveData>(WIFI_ROVER_STATUS_PORT);
    commandsClient = std::make_unique<TransmitData>(HOST_IP, WIFI_ROVER_COMMANDS_PORT);

    imageClient = std::make_unique<TransmitData>(LOCAL_IP, BASE_STATION_IMAGE_PORT);
    slamClient = std::make_unique<TransmitData>(LOCAL_IP, BASE_STATION_SLAM_PORT);
    statusClient = std::make_unique<TransmitData>(LOCAL_IP, BASE_STATION_STATUS_PORT);
    commandsServer = std::make_unique<ReceiveData>(BASE_STATION_COMMANDS_PORT);

    cameraBuffer = std::make_unique<char[]>(IMAGE_HEIGHT * IMAGE_WIDTH * 3);
    slamBuffer = std::make_unique<char[]>(IMAGE_HEIGHT * IMAGE_WIDTH);
    statusBuffer = std::make_unique<char[]>(200);
    commandsBuffer = std::make_unique<char[]>(200); 
}

void getCameraData()
{
    size_t bufferSize = IMAGE_HEIGHT * IMAGE_WIDTH * 3;
    int returnVal = 0;
    while(returnVal == 0)
    {
        returnVal = imageServer->getData(static_cast<void*>(cameraBuffer.get()), bufferSize);
    }
    imageClient->sendPayload(static_cast<void*>(cameraBuffer.get()), returnVal);
}

void getSlamData()
{
    size_t bufferSize = IMAGE_HEIGHT * IMAGE_WIDTH;
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = slamServer->getData(static_cast<void*>(slamBuffer.get()), bufferSize);
    slamClient->sendPayload(static_cast<void*>(slamBuffer.get()), returnVal);
}

void getRoverStatus()
{
    size_t bufferSize = sizeof(char) * 200;
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = statusServer->getData(static_cast<void*>(statusBuffer.get()), bufferSize);
    statusClient->sendPayload(static_cast<void*>(statusBuffer.get()), returnVal); 
}

void sendRoverCommands()
{
    size_t bufferSize = sizeof(char) * 200;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = commandsServer->getData(static_cast<void*>(commandsBuffer.get()), bufferSize);
    }
    commandsClient->sendPayload(static_cast<void*>(commandsBuffer.get()), incomingSize);
}


#endif