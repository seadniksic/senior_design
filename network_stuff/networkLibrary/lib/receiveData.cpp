#ifndef RECEIVEDATA_CPP
#define RECEIVEDATA_CPP

#include "receiveData.h"
#include <iostream>

template <class PayloadType>
ReceiveData<PayloadType>::ReceiveData(uint16_t port)
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

template <class PayloadType>
bool ReceiveData<PayloadType>::availableDataServer()
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

template <class PayloadType>
bool ReceiveData<PayloadType>::availableDataClient()
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

template <class PayloadType>
int ReceiveData<PayloadType>::getData(PayloadType *buffer, size_t bufferLength)
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
            timeout.tv_sec = 0; 
            timeout.tv_usec = 67000;
            setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        }
    }
    else
    {
        //If already connected, check to see if there is data waiting
        if(availableDataClient())
        {
            int receivedBytes = 0;
            while(receivedBytes < bufferLength)
            {
                int receiveValue = recv(clientSocket, &buffer[receivedBytes], min(MAX_PACKET_SIZE, bufferLength - receivedBytes), 0);
                if(receiveValue < 0)
                {
                    close(clientSocket);
                    clientSocket = -1;
                    break;
                }
                receivedBytes += receiveValue;
            }
            return receivedBytes;
        }
    }

    return 0;
}

template <class PayloadType>
ReceiveData<PayloadType>::~ReceiveData()
{
    close(serverSocket);
    close(clientSocket);
}

template<class PayloadType>
size_t ReceiveData<PayloadType>::min(size_t a, size_t b)
{
    if(a < b)
        return a;
    return b;
}

#endif