#ifndef RECEIVEDATA_H
#define RECEIVEDATA_H

#include <iostream>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <opencv2/opencv.hpp>
#include "common.h"

template <class PayloadType>
class ReceiveData{
    private:
        int serverSocket, clientSocket;
        struct sockaddr_in serverAddress;
        size_t min(size_t a, size_t b);
        bool availableDataServer();
        bool availableDataClient();
    public:
        ReceiveData(uint16_t port);
        int getData(PayloadType *buffer, size_t bufferLength);
        ~ReceiveData();
};

template class ReceiveData<int>;
template class ReceiveData<cv::Mat>;
template class ReceiveData<char>;
template class ReceiveData<unsigned char>;
#endif