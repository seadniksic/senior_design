#ifndef TRANSMITDATA_H
#define TRANSMITDATA_H
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

template <typename PayloadType>
class TransmitData{
    private:
        int sock;
        struct sockaddr_in serverAddress;
    public:
        TransmitData(const char *ipAddress, uint16_t port);
        int sendPayload(PayloadType *payLoad, size_t dataLength);
        ~TransmitData();
};

#endif