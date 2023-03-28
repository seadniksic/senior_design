#include "passThroughWire.h"
#include <pthread.h>
#include <iostream>
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
    pthread_join(cameraPositionThread, NULL);

    return 0;
}

void* cameraThreadFunction(void* arg)
{
    PassThroughWire connection(JETSON_IMAGE_PORT, WIFI_IMAGE_PORT, CLIENT_IP, IMAGE_BUFFER_SIZE, "Camera Feed Wire", IMAGE_DEBUG);
    while(1)
        connection.update();
    return NULL;
}

void* pointCloudThreadFunction(void* arg)
{
    PassThroughWire connection(JETSON_POINT_CLOUD_PORT, WIFI_POINT_CLOUD_PORT, CLIENT_IP, POINT_CLOUD_BUFFER_SIZE, "Point Cloud Wire", POINT_CLOUD_DEBUG);
    while(1)
        connection.update();
    return NULL;
}

void* statusThreadFunction(void* arg)
{
    PassThroughWire connection(JETSON_STATUS_PORT, WIFI_ROVER_STATUS_PORT, CLIENT_IP, ROVER_STATUS_BUFFER_SIZE, "Rover Status Wire", STATUS_DEBUG);
    while(1)
        connection.update();
    return NULL;
}

void* commandsThreadFunction(void* arg)
{
    PassThroughWire connection(WIFI_ROVER_COMMANDS_PORT, JETSON_COMMANDS_PORT, LOCAL_IP, ROVER_COMMANDS_BUFFER_SIZE, "Rover Commands Wire", COMMANDS_DEBUG);
    while(1)
        connection.update();
    return NULL;
}

void* cameraPositionThreadFunction(void *arg)
{
    PassThroughWire connection(JETSON_CAMERA_LOCATION_PORT, WIFI_CAMERA_LOCATION_PORT, CLIENT_IP, CAMERA_POSITION_BUFFER_SIZE, "Camera Position Wire", POISITON_DEBUG);
    while(1)
        connection.update();
    return NULL;
}