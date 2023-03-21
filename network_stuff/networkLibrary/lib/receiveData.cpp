#ifndef RECEIVEDATA_CPP
#define RECEIVEDATA_CPP

#include "receiveData.h"
#include <iostream>

ReceiveData::ReceiveData(uint16_t port)
{
    //Create socket using TCP protocol
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
        std::cerr << "Failed to create socket." << std::endl;

    //Set socket buffer size to 1036800
    int recv_buff_size = 1036800; // in bytes
    if(setsockopt(serverSocket, SOL_SOCKET, SO_RCVBUF, &recv_buff_size, sizeof(recv_buff_size)) == -1)
        std::cerr << "Failed to set socket buffer size" << std::endl;
    
    //Set the IP and port 
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    //Bind the port to the socket
    if (bind(serverSocket, (struct sockaddr *) &serverAddress, sizeof(serverAddress)) == -1)
        std::cerr << "Failed to bind socket to address." << std::endl;

    //Mark the socket as a listener
    if(listen(serverSocket, 2) == -1)
        std::cerr << "Failed to set up listener." << std::endl;
    
    memset(pollList, 0, sizeof(pollList));
    pollList[0].fd = serverSocket;
    pollList[0].events = POLLIN;
    
    clientSocket = -1;
}

bool ReceiveData::availableDataServer()
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(serverSocket, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int retval = select(serverSocket + 1, &rfds, NULL, NULL, &tv);
    if(retval < 0)
        return false;
    return retval;
}

bool ReceiveData::availableDataClient()
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(clientSocket, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int retval = select(clientSocket + 1, &rfds, NULL, NULL, &tv);
    if(retval < 0)
        return false;
    return retval;
}

int ReceiveData::getData(void *buffer, size_t bufferLength)
{
    int pollCount = poll(pollList, 1, 0);
    if(pollCount < 0)
        return 0;
    //Check to see if there is an active connection
    if(clientSocket < 0)
    {
        //If there is not an active connection, check to see if there are any in the queue and create connection if there is one
        if(pollList[0].revents & POLLIN)
        {
            struct sockaddr_in clientAddress;
            socklen_t clientAddressLength = sizeof(clientAddress);
            clientSocket = accept(serverSocket, (struct sockaddr*) &clientAddress, &clientAddressLength);
            if(clientSocket == -1)
                std::cerr << "Failed to accept socket connection." << std::endl;
            
            struct timeval timeout;
            timeout.tv_sec = 5; 
            timeout.tv_usec = 0;
            setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
            pollList[0].fd = clientSocket;
            pollList[0].events = POLLIN;
        }
    }
    else
    {
        //If already connected, check to see if there is data waiting
        if(pollList[0].revents & POLLIN)
        {
            int receivedBytes = 0;
            uint64_t receivingPacketLength = 0;

            int receiveValue = recv(clientSocket, &receivingPacketLength, sizeof(receivingPacketLength), 0);
            
            if(receiveValue == 0)
            {
                std::cout << "Closing socket due to bad size read" << std::endl;
                close(clientSocket);
                clientSocket = -1;
                pollList[0].fd = serverSocket;
                pollList[0].events = POLLIN;
                return 0;
            }

            std::cout << "Attempting to Receive: " << receivingPacketLength << " into " << bufferLength << std::endl;

            while(receivedBytes < bufferLength && receivedBytes < receivingPacketLength)
            {
                receiveValue = recv(clientSocket, ((char *)buffer + receivedBytes * sizeof(char)), min(min(MAX_PACKET_SIZE, bufferLength - receivedBytes), receivingPacketLength - receivedBytes), 0);
                if(receiveValue <= 0)
                {
                    std::cout << "Closing socket due to bad data read" << std::endl;
                    close(clientSocket);
                    clientSocket = -1;
                    pollList[0].fd = serverSocket;
                    pollList[0].events = POLLIN;
                    break;
                }
                receivedBytes += receiveValue;
            }
            return receivedBytes;
        }
    }

    return 0;
}

ReceiveData::~ReceiveData()
{
    close(serverSocket);
    close(clientSocket);
}

size_t ReceiveData::min(size_t a, size_t b)
{
    if(a < b)
        return a;
    return b;
}

#endif