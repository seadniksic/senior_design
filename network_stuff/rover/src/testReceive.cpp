#include <iostream>
#include "receiveData.h"
#include "common.h"

int main()
{
    ReceiveData<char> testReceive(8080);
    char buffer[1024];
    int bytesReceived;
    while(1)
    {
        bytesReceived = testReceive.getData(buffer, 1024);
        buffer[bytesReceived] = '\0';
        std::cout << buffer << std::endl;
    }
}