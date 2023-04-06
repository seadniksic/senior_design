#ifndef TRANSMITDATA_H
#define TRANSMITDATA_H

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include "common.h"
#include <sys/select.h>
#include <iconv.h>
#include <cstring>
#include <iostream>

class TransmitData{
    private:
        int sock;
        struct sockaddr_in serverAddress;
        size_t min(size_t a, size_t b);
        bool currentlyConnected;
        std::string name;
        uint8_t priority;
    public:
        TransmitData(const char *ipAddress, uint16_t port, std::string name, const uint8_t priority);
        int sendPayload(const void *payLoad, size_t dataLength);
        ~TransmitData();
};

#endif