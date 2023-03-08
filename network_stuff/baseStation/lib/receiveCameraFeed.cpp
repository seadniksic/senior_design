#ifndef RECEIVECAMERAFEED_CPP
#define RECEIVECAMERAFEED_CPP

#include "receiveCameraFeed.h"

ReceiveData<image_t> *server;

void initServer(uint16_t port)
{
    server = new ReceiveData<image_t>(port);
}

int getImageData(image_t *buffer, size_t bufferSize)
{
    int error = server->getData(buffer, bufferSize);
    return error;
}

void turnOffServer()
{
    delete server;
}

#endif