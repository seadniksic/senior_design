#ifndef RECEIVEDATA_CPP
#define RECEIVEDATA_CPP

#include "receiveData.h"

template <typename PayloadType>
ReceiveData<PayloadType>::ReceiveData(uint16_t port)
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
        std::cerr << "Failed to create socket." << std::endl;

    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    int bindResult = bind(sock, (struct sockaddr *) &serverAddress, sizeof(serverAddress));
    if (bindResult == -1)
        std::cerr << "Failed to bind socket to address." << std::endl;
}

template <typename PayloadType>
int ReceiveData<PayloadType>::getData(PayloadType *buffer)
{
    int bytesReceived = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if(bytesReceived == -1)
        std::cerr << "Failed to receive data." << std::endl;
    
    return bytesReceived;
}

template <typename PayloadType>
ReceiveData<PayloadType>::~ReceiveData()
{
    close(sock);
}

#endif