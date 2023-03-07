#ifndef RECEIVEDATA_H
#define RECEIVEDATA_H

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "common.h"

template <class PayloadType>
class ReceiveData{
    private:
        int sock;
        struct sockaddr_in serverAddress;
        size_t min(size_t a, size_t b);
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