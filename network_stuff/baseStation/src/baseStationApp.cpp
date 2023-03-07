#include "receiveCameraFeed.h"
#include <iostream>
#include "common.h"

int main()
{
    initServer(WIFI_IMAGE_PORT);
    cv::Mat *buffer = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

    while(1)
    {
        getImageData(buffer->data, buffer->total() * buffer->elemSize());
        cv::imshow("Display window", *buffer);
        int k = cv::waitKey(0); // Wait for a keystroke in the window
    }

    turnOffServer();
    return 0;
}