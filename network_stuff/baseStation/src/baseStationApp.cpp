#include "receiveCameraFeed.h"
#include <iostream>

int main()
{
    initServer(8080);
    image_t *buffer;

    while(1)
    {
        getImageData(buffer);
        std::cout << "Here: " << std::endl;
    }

    turnOffServer();
    return 0;
}