#ifndef RECEIVEDATA_H
#define RECEIVEDATA_H

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

template <class PayloadType>
class ReceiveData{
    private:
        int sock;
        struct sockaddr_in serverAddress;
    public:
        ReceiveData(uint16_t port);
        int getData(PayloadType *buffer);
        ~ReceiveData();
};

template class ReceiveData<int>;

template class ReceiveData<cv::Mat>;
#endif