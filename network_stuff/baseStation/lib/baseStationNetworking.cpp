#ifndef BASESTATIONNETWORKING_CPP
#define BASESTATIONNETWORKING_CPP

#include "baseStationNetworking.h"

std::unique_ptr<ReceiveData> imageServer, pointCloudServer, statusServer, commandsServer, cameraLocalizationServer;
std::unique_ptr<TransmitData> imageClient, pointCloudClient, statusClient, commandsClient, cameraLocalizationClient;
std::unique_ptr<char[]> cameraBuffer, pointCloudBuffer, statusBuffer, commandsBuffer, cameraLocalizationBuffer;

void initializeNetwork()
{
    imageServer = std::make_unique<ReceiveData>(WIFI_IMAGE_PORT);
    pointCloudServer = std::make_unique<ReceiveData>(WIFI_POINT_CLOUD_PORT);
    statusServer = std::make_unique<ReceiveData>(WIFI_ROVER_STATUS_PORT);
    commandsClient = std::make_unique<TransmitData>(HOST_IP, WIFI_ROVER_COMMANDS_PORT);
    cameraLocalizationServer = std::make_unique<ReceiveData>(WIFI_CAMERA_LOCATION_PORT);

    imageClient = std::make_unique<TransmitData>(LOCAL_IP, BASE_STATION_IMAGE_PORT);
    pointCloudClient = std::make_unique<TransmitData>(LOCAL_IP, BASE_STATION_POINT_CLOUD_PORT);
    statusClient = std::make_unique<TransmitData>(LOCAL_IP, BASE_STATION_STATUS_PORT);
    commandsServer = std::make_unique<ReceiveData>(BASE_STATION_COMMANDS_PORT);
    cameraLocalizationClient = std::make_unique<TransmitData>(LOCAL_IP, BASE_STATION_CAMERA_LOCATION_PORT);

    cameraBuffer = std::make_unique<char[]>(IMAGE_BUFFER_SIZE);
    pointCloudBuffer = std::make_unique<char[]>(POINT_CLOUD_BUFFER_SIZE);
    statusBuffer = std::make_unique<char[]>(ROVER_STATUS_BUFFER_SIZE);
    commandsBuffer = std::make_unique<char[]>(ROVER_COMMANDS_BUFFER_SIZE); 
    cameraLocalizationBuffer = std::make_unique<char[]>(CAMERA_POSITION_BUFFER_SIZE);
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

void getPointCloudData()
{
    int returnVal = 0;
    while(returnVal == 0)
        returnVal = pointCloudServer->getData(static_cast<void*>(pointCloudBuffer.get()), POINT_CLOUD_BUFFER_SIZE);
    pointCloudClient->sendPayload(static_cast<void*>(pointCloudBuffer.get()), returnVal);
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
        incomingSize = commandsServer->getData(static_cast<void*>(commandsBuffer.get()), ROVER_COMMANDS_BUFFER_SIZE);
    commandsClient->sendPayload(static_cast<void*>(commandsBuffer.get()), incomingSize);
}

void getCameraPosition()
{
    int incomingSize = 0;
    while(incomingSize = 0)
        incomingSize = cameraLocalizationServer->getData(static_cast<void*>(cameraLocalizationBuffer.get()), CAMERA_POSITION_BUFFER_SIZE);
    cameraLocalizationClient->sendPayload(static_cast<void*>(cameraLocalizationBuffer.get()), incomingSize);
}

#endif