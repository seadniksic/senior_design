#include "roverNetworking.h"
#include <pthread.h>

pthread_t cameraThread, slamThread, statusThread, commandsThread;

void* cameraThreadFunction(void* arg);
void* slamThreadFunction(void* arg);
void* statusThreadFunction(void* arg);
void* commandsThreadFunction(void* arg);
void exitHandler(void);

int main()
{
    atexit(exitHandler);
    initializeNetwork();

    pthread_create(&cameraThread, NULL, cameraThreadFunction, NULL);
    pthread_create(&slamThread, NULL, slamThreadFunction, NULL);
    pthread_create(&statusThread, NULL, statusThreadFunction, NULL);
    pthread_create(&commandsThread, NULL, commandsThreadFunction, NULL);

    pthread_join(cameraThread, NULL);
    pthread_join(slamThread, NULL);
    pthread_join(statusThread, NULL);
    pthread_join(commandsThread, NULL);

    shutdownNetwork();

    return 0;
}

void* cameraThreadFunction(void* arg)
{
    while(1)
    {
        sendCameraData();
    }

    return NULL;
}

void* slamThreadFunction(void* arg)
{
    while(1)
    {
        sendSlamData();
    }

    return NULL;
}

void* statusThreadFunction(void* arg)
{
    while(1)
    {
        sendRoverStatus();
    }

    return NULL;
}

void* commandsThreadFunction(void* arg)
{
    while(1)
    {
        getRoverCommands();
    }

    return NULL;
}

void exitHandler(void)
{
    pthread_cancel(cameraThread);
    pthread_cancel(slamThread);
    pthread_cancel(statusThread);
    pthread_cancel(commandsThread);
}