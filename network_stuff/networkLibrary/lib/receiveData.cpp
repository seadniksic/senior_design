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
    //Check to see if there is an active connection
    if(clientSocket < 0)
    {
        //If there is not an active connection, check to see if there are any in the queue and create connection if there is one
        if(availableDataServer())
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
        }
    }
    else
    {
        //If already connected, check to see if there is data waiting
        if(availableDataClient())
        {
            int receivedBytes = 0;
            uint64_t receivingPacketLength = 0;

            char firstPacket[100];
            int receiveValue = recv(clientSocket, firstPacket, 100, 0);
            receivingPacketLength = atoi(firstPacket);

            std::cout << "Attempting to Recieve: " << receivingPacketLength << " into " << bufferLength << std::endl;
            
            if(receiveValue == 0)
            {
                close(clientSocket);
                clientSocket = -1;
            }
            receiveValue = 0;
            while(receivedBytes < bufferLength && receivedBytes < receivingPacketLength)
            {
                std::cout << "Before Read: " << receivedBytes << std::endl; 
                receiveValue = recv(clientSocket, ((char *)buffer + receivedBytes * sizeof(char)), min(MAX_PACKET_SIZE, bufferLength - receivedBytes), 0);
                if(receiveValue <= 0)
                {
                    close(clientSocket);
                    clientSocket = -1;
                    break;
                }
                receivedBytes += receiveValue;
                std::cout << "After Read: " << receivedBytes << std::endl;
            }
            std::cout << availableDataClient() << std::endl;
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