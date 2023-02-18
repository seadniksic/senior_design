#include "receiveData.h"
#include "transmitData.h"
#include <iostream>

int main()
{
    ReceiveData<int> newData(8080);
    TransmitData<int> sentData("10.0.0.203", 8080);

    int buffer;
    int sender = 10;
    sentData.sendPayload(&sender, 1);
    newData.getData(&buffer);

}