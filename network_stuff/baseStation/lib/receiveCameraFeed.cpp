#ifndef RECEIVECAMERAFEED_CPP
#define RECEIVECAMERAFEED_CPP

#include "receiveCameraFeed.h"

ReceiveData<image_t> *server;

void initServer(uint16_t port)
{
    server = new ReceiveData<image_t>(port);
}

void getImageData(image_t *buffer, size_t bufferSize)
{
    int error = server->getData(buffer, bufferSize);
}

bool checkAvailableData()
{
    return server->availableData();
}

void turnOffServer()
{
    delete server;
}

#endif