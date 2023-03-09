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

    if(connect(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
        std::cerr << "Failed to connect to server." << std::endl;
}

template <class PayloadType>
int TransmitData<PayloadType>::sendPayload(PayloadType *payLoad, size_t dataLength)
{
    int sentBytes = 0;
    while(sentBytes < dataLength)
    {
        int bytesToSend = min(MAX_PACKET_SIZE, dataLength - sentBytes);
        int sendData = send(sock, &payLoad[sentBytes], bytesToSend, 0);
        if(sendData == -1)
        {
            std::cerr << "Failed to send data due to " << errno << std::endl;
            return 1;
        }
        sentBytes += bytesToSend;
    }
    return 0;
}

template <class PayloadType>
TransmitData<PayloadType>::~TransmitData()
{
    close(sock);
}

template <class PayloadType>
size_t TransmitData<PayloadType>::min(size_t a, size_t b)
{
    if(a < b)
        return a;
    return b;
}

#endif