#ifndef RECEIVEDATA_H
#define RECEIVEDATA_H
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>


template <typename PayloadType>
class ReceiveData{
    private:
        int sock;
        struct sockaddr_in serverAddress;
    public:
        ReceiveData(uint16_t port);
        int getData(PayloadType *buffer);
        ~ReceiveData();
};

#endif