#ifndef TRANSMITDATA_CPP
#define TRANSMITDATA_CPP
#include "transmitData.h"


template <class PayloadType>
TransmitData<PayloadType>::TransmitData(const char *ipAddress, uint16_t port)
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock == -1)
        std::cerr << "Failed to create socket." << std::endl;

    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    serverAddress.sin_addr.s_addr = inet_addr(ipAddress);
}

template <class PayloadType>
int TransmitData<PayloadType>::sendPayload(PayloadType *payLoad, size_t dataLength)
{
    int sendData = sendto(sock, payLoad, dataLength, 0, (struct sockaddr *) &serverAddress, sizeof(serverAddress));
    if(sendData == -1)
    {
        std::cerr << "Failed to send data." << std::endl;
        return 1;
    }

    return 0;
}

template <class PayloadType>
TransmitData<PayloadType>::~TransmitData()
{
    close(sock);
}

#endif