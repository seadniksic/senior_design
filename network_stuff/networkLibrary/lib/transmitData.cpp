#ifndef TRANSMITDATA_CPP
#define TRANSMITDATA_CPP
#include "transmitData.h"

TransmitData::TransmitData(const char *ipAddress, uint16_t port, std::string name)
{
    this->name = name;
    //Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1)
        std::cerr << "Failed to create socket for " << name << "..." << std::endl;

    //Mark the IP address it should be connecting to
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    serverAddress.sin_addr.s_addr = inet_addr(ipAddress);

    currentlyConnected = false;
}

int TransmitData::sendPayload(const void *payLoad, size_t dataLength)
{
    int sentBytes = 0;
    //Check to see if the socket is currently connected to anything
    if(!currentlyConnected)
    {
        //If not try to connect
        if(connect(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) != -1)
            currentlyConnected = true;
    }
    if(currentlyConnected)
    {
        //If it is conected send the data
        uint64_t sendDataSize = (uint64_t)dataLength;

        int sendResult = send(sock, &sendDataSize, sizeof(sendDataSize), MSG_NOSIGNAL), bytesToSend = 0;

        while(sentBytes < dataLength)
        {
            bytesToSend = min(MAX_PACKET_SIZE, dataLength - sentBytes);
            sendResult = send(sock, ((char *)payLoad + sentBytes * sizeof(char)), bytesToSend, MSG_NOSIGNAL);
            if(sendResult == -1)
            {
                //If data failed to send due to connection failure, try and reconnect
                if(errno == EPIPE || errno == ENOTCONN)
                {
                    currentlyConnected = false;
                    close(sock);
                    sock = socket(AF_INET, SOCK_STREAM, 0);
                    if(sock == -1)
                        std::cerr << "Failed to create socket for " << name << "..." << std::endl;
                    break;
                }
                else
                    std::cerr << "Failed to send data for " << name << "..." << std::endl;
            }
            sentBytes += sendResult;
        }
    }
    return sentBytes;
}

TransmitData::~TransmitData()
{
    if(sock > -1)
        close(sock);
    std::cout << "Successfully closed transmit of " << name << "..." << std::endl;
}

size_t TransmitData::min(size_t a, size_t b)
{
    if(a < b)
        return a;
    return b;
}

#endif