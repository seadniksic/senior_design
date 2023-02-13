#ifndef RECEIVECAMERAFEED_CPP
#define RECEIVECAMERAFEED_CPP

#include "receiveCameraFeed.h"

ReceiveData<image_t> *server;

void initServer(uint16_t port)
{
    server = new ReceiveData<image_t>(port);
}

void getImageData(image_t *buffer)
{
    int error = server->getData(buffer);
}

void turnOffServer()
{
    delete server;
}

#endif