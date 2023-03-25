#ifndef RECEIVEDATA_H
#define RECEIVEDATA_H

#include <iostream>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "common.h"
#include <cstring>
#include <poll.h>

class ReceiveData{
    private:
        int serverSocket, clientSocket;
        struct sockaddr_in serverAddress;
        size_t min(size_t a, size_t b);
        struct pollfd pollList[1];
        bool availableDataServer();
        bool availableDataClient();
        std::string name;
    public:
        ReceiveData(uint16_t port, std::string name);
        int getData(void *buffer, size_t bufferLength);
        ~ReceiveData();
};

#endif