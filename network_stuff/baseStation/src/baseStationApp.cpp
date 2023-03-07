#include "receiveCameraFeed.h"
#include <iostream>
#include "common.h"

int main()
{
    initServer(WIFI_IMAGE_PORT);
    image_t *buffer = new image_t(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

    while(1)
    {
        getImageData(buffer, buffer->total());
        std::cout << "Here: " << std::endl;
    }

    turnOffServer();
    return 0;
}