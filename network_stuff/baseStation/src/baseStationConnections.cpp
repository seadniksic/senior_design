#include "baseStationNetworking.h"
#include <pthread.h>
#include <csignal>

pthread_t cameraThread, pointCloudThread, statusThread, commandsThread, cameraPositionThread;

void* cameraThreadFunction(void* arg);
void* pointCloudThreadFunction(void* arg);
void* statusThreadFunction(void* arg);
void* commandsThreadFunction(void* arg);
void* cameraPositionThreadFunction(void* arg);
void exitHandler(int signal_number)
{
    std::cout << "\nExit signal received, shutting down threads..." << std::endl;
    pthread_cancel(cameraThread);
    pthread_cancel(pointCloudThread);
    pthread_cancel(statusThread);
    pthread_cancel(commandsThread);
    pthread_cancel(cameraPositionThread);
    std::cout << "All threads closed, exiting program..." << std::endl; 
    exit(0);
}

int main()
{
    signal(SIGABRT, exitHandler);
    signal(SIGTERM, exitHandler);
    signal(SIGINT, exitHandler);

    std::cout << "Initializing network..." << std::endl;

    initializeNetwork();

    std::cout << "Successfully initialized network..." << std::endl;
    std::cout << "Launching network threads..." << std::endl;

    pthread_create(&cameraThread, NULL, cameraThreadFunction, NULL);
    pthread_create(&pointCloudThread, NULL, pointCloudThreadFunction, NULL);
    pthread_create(&statusThread, NULL, statusThreadFunction, NULL);
    pthread_create(&commandsThread, NULL, commandsThreadFunction, NULL);
    pthread_create(&cameraPositionThread, NULL, cameraPositionThreadFunction, NULL);

    std::cout << "Successfully launched network threads..." << std::endl;

    pthread_join(cameraThread, NULL);
    pthread_join(pointCloudThread, NULL);
    pthread_join(statusThread, NULL);
    pthread_join(commandsThread, NULL);

    return 0;
}

void* cameraThreadFunction(void* arg)
{
    while(1)
        getCameraData();
    return NULL;
}

void* pointCloudThreadFunction(void* arg)
{
    while(1)
    {
        getPointCloudData();
    }

    return NULL;
}

void* statusThreadFunction(void* arg)
{
    while(1)
        getRoverStatus();
    return NULL;
}

void* commandsThreadFunction(void* arg)
{
    while(1)
        sendRoverCommands();
    return NULL;
}

void* cameraPositionThreadFunction(void* arg)
{
    while(1)
        getCameraPosition();
    return NULL;
}