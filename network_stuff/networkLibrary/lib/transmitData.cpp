#ifndef TRANSMITDATA_CPP
#define TRANSMITDATA_CPP
#include "transmitData.h"

template <class PayloadType>
TransmitData<PayloadType>::TransmitData(const char *ipAddress, uint16_t port)
{
    //Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1)
        std::cerr << "Failed to create socket." << std::endl;

    //Mark the IP address it should be connecting to
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    serverAddress.sin_addr.s_addr = inet_addr(ipAddress);

    currentlyConnected = false;
}

template <class PayloadType>
int TransmitData<PayloadType>::sendPayload(PayloadType *payLoad, size_t dataLength)
{
    //Check to see if the socket is currently connected to anything
    if(!currentlyConnected)
    {
        //If not try to connect
        if(connect(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) != -1)
            currentlyConnected = true;
    }
    else
    {
        //If it is conected send the data
        int sentBytes = 0;
        while(sentBytes < dataLength)
        {
            int bytesToSend = min(MAX_PACKET_SIZE, dataLength - sentBytes);
            int sendData = send(sock, &payLoad[sentBytes], bytesToSend, 0);
            if(sendData == -1)
            {
                //If data failed to send due to connection failure, try and reconnect
                if(errno == EPIPE || errno == ENOTCONN)
                {
                    currentlyConnected = false;
                    break;
                }
                else
                    std::cerr << "Failed to send data." << std::endl;
            }
            sentBytes += bytesToSend;
        }
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