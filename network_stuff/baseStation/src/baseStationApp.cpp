#include "receiveCameraFeed.h"

int main()
{
    initServer(8080);
    image_t *buffer;

    while(1)
    {
        getImageData(buffer);
    }

    turnOffServer();
    return 0;
}