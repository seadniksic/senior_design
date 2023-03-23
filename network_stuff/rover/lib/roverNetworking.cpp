#ifndef ROVERNETWORKING_CPP
#define ROVERNETWORKING_CPP

#include "roverNetworking.h"

std::unique_ptr<ReceiveData> imagePortServer, slamPortServer, statusPortServer, commandsServer;
std::unique_ptr<TransmitData> imageClient, slamClient, statusClient, commandsPortClient;
std::unique_ptr<char[]> cameraBuffer, slamBuffer, statusBuffer, commandsBuffer;

void initializeNetwork()
{
    imageClient = std::make_unique<TransmitData>(CLIENT_IP, WIFI_IMAGE_PORT);
    slamClient = std::make_unique<TransmitData>(CLIENT_IP, WIFI_SLAM_PORT);
    statusClient = std::make_unique<TransmitData>(CLIENT_IP, WIFI_ROVER_STATUS_PORT);
    commandsServer = std::make_unique<ReceiveData>(WIFI_ROVER_STATUS_PORT);

    imagePortServer = std::make_unique<ReceiveData>(JETSON_IMAGE_PORT);
    slamPortServer = std::make_unique<ReceiveData>(JETSON_SLAM_PORT);
    statusPortServer = std::make_unique<ReceiveData>(JETSON_STATUS_PORT);
    commandsPortClient = std::make_unique<TransmitData>(LOCAL_IP, JETSON_COMMANDS_PORT);

    cameraBuffer = std::make_unique<char[]>(IMAGE_BUFFER_SIZE);
    slamBuffer = std::make_unique<char[]>(POINT_CLOUD_BUFFER_SIZE);
    statusBuffer = std::make_unique<char[]>(ROVER_STATUS_BUFFER_SIZE);
    commandsBuffer = std::make_unique<char[]>(ROVER_COMMANDS_BUFFER_SIZE); 
}   

void sendCameraData()
{
    int incomingSize = 0;
    while(incomingSize == 0)
        incomingSize = imagePortServer->getData(static_cast<void*>(cameraBuffer.get()), IMAGE_BUFFER_SIZE);
    imageClient->sendPayload(static_cast<void*>(cameraBuffer.get()), incomingSize);
}

void sendSlamData()
{
    int incomingSize = 0;
    while(incomingSize == 0)
        incomingSize = slamPortServer->getData(static_cast<void*>(slamBuffer.get()), POINT_CLOUD_BUFFER_SIZE);
    slamClient->sendPayload(static_cast<void*>(slamBuffer.get()), incomingSize);
}

void sendRoverStatus()
{
    int incomingSize = 0;
    while(incomingSize == 0)
        incomingSize = statusPortServer->getData(static_cast<void*>(statusBuffer.get()), ROVER_STATUS_BUFFER_SIZE);
    statusClient->sendPayload(static_cast<void*>(statusBuffer.get()), incomingSize);
}

void getRoverCommands()
{
    int incomingSize = 0;
    while(incomingSize == 0)
        incomingSize = commandsServer->getData(static_cast<void*>(commandsBuffer.get()), ROVER_COMMANDS_BUFFER_SIZE);
    commandsPortClient->sendPayload(static_cast<void*>(commandsBuffer.get()), incomingSize);
}


#endif