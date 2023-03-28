#ifndef TRANSMITDATA_H
#define TRANSMITDATA_H

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <errno.h>
#include "common.h"
#include <sys/select.h>
#include <iconv.h>
#include <cstring>

class TransmitData{
    private:
        int sock;
        struct sockaddr_in serverAddress;
        size_t min(size_t a, size_t b);
        bool currentlyConnected;
    public:
        TransmitData(const char *ipAddress, uint16_t port);
        int sendPayload(void *payLoad, size_t dataLength);
        ~TransmitData();
};

#endif