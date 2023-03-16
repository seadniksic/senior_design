#include "transmitData.h"
#include <unistd.h>
#include "common.h"

int main()
{
    TransmitData testTransmit(CLIENT_IP, 8080);
    char payload[] = "Hello World";
    while(1)
    {
        testTransmit.sendPayload(payload, sizeof(payload) / sizeof(payload[0]));
        sleep(2);
    }
    return 0;
}