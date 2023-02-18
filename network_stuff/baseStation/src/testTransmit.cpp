#include "transmitData.h"
#include <unistd.h>

int main()
{
    TransmitData<char> testTransmit("10.42.0.1", 8080);
    while(1)
    {
        testTransmit.sendPayload("Hello World\n", 13);
        sleep(2);
    }
    return 0;
}