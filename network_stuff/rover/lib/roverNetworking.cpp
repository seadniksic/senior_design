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

    cameraBuffer = std::make_unique<char[]>(IMAGE_HEIGHT * IMAGE_WIDTH * 3);
    slamBuffer = std::make_unique<char[]>(IMAGE_HEIGHT * IMAGE_WIDTH);
    statusBuffer = std::make_unique<char[]>(200);
    commandsBuffer = std::make_unique<char[]>(200); 
}   

void sendCameraData()
{
    size_t bufferSize = sizeof(char) * IMAGE_HEIGHT * IMAGE_WIDTH * 3;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = imagePortServer->getData(static_cast<void*>(cameraBuffer.get()), bufferSize);
    }
    imageClient->sendPayload(static_cast<void*>(cameraBuffer.get()), incomingSize);
}

void sendSlamData()
{
    size_t bufferSize = sizeof(char) * IMAGE_HEIGHT * IMAGE_WIDTH;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = slamPortServer->getData(static_cast<void*>(slamBuffer.get()), bufferSize);
    }
    slamClient->sendPayload(static_cast<void*>(slamBuffer.get()), incomingSize);
}

void sendRoverStatus()
{
    size_t bufferSize = sizeof(char) * 200;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = statusPortServer->getData(static_cast<void*>(statusBuffer.get()), bufferSize);
    }
    statusClient->sendPayload(static_cast<void*>(statusBuffer.get()), incomingSize);
}

void getRoverCommands()
{
    size_t bufferSize = sizeof(char) * 200;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = commandsServer->getData(static_cast<void*>(commandsBuffer.get()), bufferSize);
    }
    commandsPortClient->sendPayload(static_cast<void*>(commandsBuffer.get()), incomingSize);
}


#endif