#ifndef ROVERNETWORKING_CPP
#define ROVERNETWORKING_CPP

#include "roverNetworking.h"


TransmitData *imageClient;
TransmitData *slamClient;
TransmitData *statusClient;
ReceiveData *commandsServer;

ReceiveData *imagePortServer;
ReceiveData *slamPortServer;
ReceiveData *statusPortServer;
TransmitData *commandsPortClient;

void initializeNetwork()
{
    imageClient = new TransmitData(CLIENT_IP, WIFI_IMAGE_PORT);
    slamClient = new TransmitData(CLIENT_IP, WIFI_SLAM_PORT);
    statusClient = new TransmitData(CLIENT_IP, WIFI_ROVER_STATUS_PORT);
    commandsServer = new ReceiveData(WIFI_ROVER_COMMANDS_PORT);

    imagePortServer = new ReceiveData(JETSON_IMAGE_PORT);
    slamPortServer = new ReceiveData(JETSON_SLAM_PORT);
    statusPortServer = new ReceiveData(JETSON_STATUS_PORT);
    commandsPortClient = new TransmitData(HOST_IP, JETSON_COMMANDS_PORT);
}

void shutdownNetwork()
{
    delete imageClient;
    delete slamClient;
    delete statusClient;
    delete commandsServer;

    delete imagePortServer;
    delete slamPortServer;
    delete statusPortServer;
    delete commandsPortClient;
}

void sendCameraData()
{
    cv::Mat buffer(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    size_t bufferSize = buffer.total() * buffer.elemSize();

    while(imagePortServer->getData(buffer.data, bufferSize) == 0);
    imageClient->sendPayload(buffer.data, bufferSize);
}

void sendSlamData()
{
    cv::Mat buffer(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    size_t bufferSize = buffer.total() * buffer.elemSize();

    while(slamPortServer->getData(buffer.data, bufferSize) == 0);
    slamClient->sendPayload(buffer.data, bufferSize);
}

void sendRoverStatus()
{
    status_t buffer;
    size_t bufferSize = sizeof(buffer);

    while(statusPortServer->getData(&buffer, bufferSize) == 0);
    statusClient->sendPayload(&buffer, bufferSize);
}

void getRoverCommands()
{
    commands_t buffer;
    size_t bufferSize = sizeof(buffer);

    while(commandsServer->getData(&buffer, bufferSize) == 0);
    std::cout << "Received Rover COmmand" << std::endl;
    commandsPortClient->sendPayload(&buffer, bufferSize);
}


#endif