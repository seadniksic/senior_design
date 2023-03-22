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

char *imageBuffer, *slamBuffer, *statusBuffer, *commandsBuffer;

void initializeNetwork()
{
    imageClient = new TransmitData(CLIENT_IP, WIFI_IMAGE_PORT);
    slamClient = new TransmitData(CLIENT_IP, WIFI_SLAM_PORT);
    statusClient = new TransmitData(CLIENT_IP, WIFI_ROVER_STATUS_PORT);
    commandsServer = new ReceiveData(WIFI_ROVER_COMMANDS_PORT);

    imagePortServer = new ReceiveData(JETSON_IMAGE_PORT);
    slamPortServer = new ReceiveData(JETSON_SLAM_PORT);
    statusPortServer = new ReceiveData(JETSON_STATUS_PORT);
    commandsPortClient = new TransmitData(LOCAL_IP, JETSON_COMMANDS_PORT);

    imageBuffer = new char[IMAGE_HEIGHT * IMAGE_WIDTH * 3];
    slamBuffer = new char[IMAGE_HEIGHT * IMAGE_WIDTH];
    statusBuffer = new char[200];
    commandsBuffer = new char[200];
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

    delete [] imageBuffer;
    delete [] slamBuffer;
    delete [] statusBuffer;
    delete [] commandsBuffer;
}

void sendCameraData()
{
    size_t bufferSize = sizeof(char) * IMAGE_HEIGHT * IMAGE_WIDTH * 3;

    int incomingSize = 0;
    while(incomingSize == 0);
    {
        incomingSize = imagePortServer->getData(imageBuffer, bufferSize);
    }
    imageClient->sendPayload(imageBuffer, incomingSize);
}

void sendSlamData()
{
    size_t bufferSize = sizeof(char) * IMAGE_HEIGHT * IMAGE_WIDTH;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = slamPortServer->getData(slamBuffer, bufferSize);
    }
    slamClient->sendPayload(slamBuffer, incomingSize);
}

void sendRoverStatus()
{
    size_t bufferSize = sizeof(char) * 200;

    int incomingSize = 0;
    while(incomingSize == 0)
    {
        incomingSize = statusPortServer->getData(statusBuffer, bufferSize);
    }
    statusClient->sendPayload(statusBuffer, incomingSize);
}

void getRoverCommands()
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
    commandsPortClient->sendPayload(commandsBuffer, incomingSize);
}


#endif