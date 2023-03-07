#include <iostream>
#include "receiveData.h"
#include "common.h"
#include <opencv2/opencv.hpp>

int main()
{
    TransmitData<char> testTransmit(CLIENT_IP, WIFI_IMAGE_PORT);

    cv::Mat img = cv::imread("testSources/testImage.jpg", IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
    while(1)
    {
        testTransmit.sendPayload(img, img.total());
    }
}