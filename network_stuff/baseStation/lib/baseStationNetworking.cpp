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

    cameraBuffer = std::make_unique<char[]>(IMAGE_BUFFER_SIZE);
    slamBuffer = std::make_unique<char[]>(POINT_CLOUD_BUFFER_SIZE);
    statusBuffer = std::make_unique<char[]>(ROVER_STATUS_BUFFER_SIZE);
    commandsBuffer = std::make_unique<char[]>(ROVER_COMMANDS_BUFFER_SIZE); 
}

void getCameraData()
{
    int returnVal = 0;
    while(returnVal == 0)
    {
        returnVal = imageServer->getData(static_cast<void*>(cameraBuffer.get()), IMAGE_BUFFER_SIZE);
    }
    imageClient->sendPayload(static_cast<void*>(cameraBuffer.get()), returnVal);
}

void getSlamData()
{
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = slamServer->getData(static_cast<void*>(slamBuffer.get()), POINT_CLOUD_BUFFER_SIZE);
    slamClient->sendPayload(static_cast<void*>(slamBuffer.get()), returnVal);
}

void getRoverStatus()
{
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = statusServer->getData(static_cast<void*>(statusBuffer.get()), ROVER_STATUS_BUFFER_SIZE);
    statusClient->sendPayload(static_cast<void*>(statusBuffer.get()), returnVal); 
}

void sendRoverCommands()
{
    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = commandsServer->getData(static_cast<void*>(commandsBuffer.get()), ROVER_COMMANDS_BUFFER_SIZE);
    }
    commandsClient->sendPayload(static_cast<void*>(commandsBuffer.get()), incomingSize);
}


#endif