#ifndef RECEIVEDATA_CPP
#define RECEIVEDATA_CPP

#include "receiveData.h"
#include <iostream>

template <class PayloadType>
ReceiveData<PayloadType>::ReceiveData(uint16_t port)
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
        std::cerr << "Failed to create socket." << std::endl;

    fcntl(sock, F_SETFL, O_NONBLOCK);
    int recv_buff_size = 1036800; // in bytes
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buff_size, sizeof(recv_buff_size));
    
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    int bindResult = bind(sock, (struct sockaddr *) &serverAddress, sizeof(serverAddress));
    if (bindResult == -1)
        std::cerr << "Failed to bind socket to address." << std::endl;
    
    currentIndex = 0;
}

template <class PayloadType>
bool ReceiveData<PayloadType>::availableData()
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int retval = select(sock + 1, &rfds, NULL, NULL, &tv);
    if(retval < 0)
        return false;
    return retval;
}

template <class PayloadType>
int ReceiveData<PayloadType>::getData(PayloadType *buffer, size_t bufferLength)
{
    int bytes_waiting;
    while(ioctl(sock, FIONREAD, &bytes_waiting) < bufferLength - 1)
    {
        receivedBytes = recv(sock, &buffer[currentIndex], min(MAX_PACKET_SIZE, bufferLength - currentIndex), 0);
        return receiveBytes;
    }

    return 0;
}

template <class PayloadType>
ReceiveData<PayloadType>::~ReceiveData()
{
    close(sock);
}

template<class PayloadType>
size_t ReceiveData<PayloadType>::min(size_t a, size_t b)
{
    if(a < b)
        return a;
    return b;
}

#endif