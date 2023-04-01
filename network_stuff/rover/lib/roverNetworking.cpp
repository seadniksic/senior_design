#ifndef ROVERNETWORKING_CPP
#define ROVERNETWORKING_CPP

#include "roverNetworking.h"

std::unique_ptr<ReceiveData> imagePortServer, pointCloudServer, statusPortServer, commandsServer, cameraPositionServer;
std::unique_ptr<TransmitData> imageClient, pointCloudClient, statusClient, commandsPortClient, cameraPositionClient;
std::unique_ptr<char[]> cameraBuffer, pointCloudBuffer, statusBuffer, commandsBuffer, cameraPositionBuffer;

void initializeNetwork()
{
    imageClient = std::make_unique<TransmitData>(CLIENT_IP, WIFI_IMAGE_PORT);
    pointCloudClient = std::make_unique<TransmitData>(CLIENT_IP, WIFI_POINT_CLOUD_PORT);
    statusClient = std::make_unique<TransmitData>(CLIENT_IP, WIFI_ROVER_STATUS_PORT);
    commandsServer = std::make_unique<ReceiveData>(WIFI_ROVER_STATUS_PORT);
    cameraPositionClient = std::make_unique<TransmitData>(CLIENT_IP, WIFI_CAMERA_LOCATION_PORT);

    imagePortServer = std::make_unique<ReceiveData>(JETSON_IMAGE_PORT);
    pointCloudServer = std::make_unique<ReceiveData>(JETSON_POINT_CLOUD_PORT);
    statusPortServer = std::make_unique<ReceiveData>(JETSON_STATUS_PORT);
    commandsPortClient = std::make_unique<TransmitData>(LOCAL_IP, JETSON_COMMANDS_PORT);
    cameraPositionServer = std::make_unique<ReceiveData>(JETSON_CAMERA_LOCATION_PORT);

    cameraBuffer = std::make_unique<char[]>(IMAGE_BUFFER_SIZE);
    pointCloudBuffer = std::make_unique<char[]>(POINT_CLOUD_BUFFER_SIZE);
    statusBuffer = std::make_unique<char[]>(ROVER_STATUS_BUFFER_SIZE);
    commandsBuffer = std::make_unique<char[]>(ROVER_COMMANDS_BUFFER_SIZE); 
    cameraPositionBuffer = std::make_unique<char[]>(CAMERA_POSITION_BUFFER_SIZE);
}   

void sendCameraData()
{
    int incomingSize = 0;
    while(incomingSize == 0)
        incomingSize = imagePortServer->getData(static_cast<void*>(cameraBuffer.get()), IMAGE_BUFFER_SIZE);
    imageClient->sendPayload(static_cast<void*>(cameraBuffer.get()), incomingSize);
}

void sendPointCloudData()
{
    int incomingSize = 0;
    while(incomingSize == 0)
        incomingSize = pointCloudServer->getData(static_cast<void*>(pointCloudBuffer.get()), POINT_CLOUD_BUFFER_SIZE);
    pointCloudClient->sendPayload(static_cast<void*>(pointCloudBuffer.get()), incomingSize);
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

void sendCameraPosition()
{
    int incomingSize = 0;
    while(incomingSize = 0)
        incomingSize = cameraPositionServer->getData(static_cast<void*>(cameraPositionBuffer.get()), CAMERA_POSITION_BUFFER_SIZE);
    cameraPositionClient->sendPayload(static_cast<void*>(cameraPositionBuffer.get()), incomingSize);
}

#endif