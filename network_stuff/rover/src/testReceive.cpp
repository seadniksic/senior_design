#include <iostream>
#include "receiveData.h"

int main()
{
    ReceiveData<char> testReceive(8080);
    char buffer[1024];
    int bytesReceived;
    while(1)
    {
        bytesReceived = testReceive.getData(buffer);
        buffer[bytesReceived] = '\0';
        std::cout << bytesReceived << std::endl;
    }
}