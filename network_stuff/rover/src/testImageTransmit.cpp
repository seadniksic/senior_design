#include <iostream>
#include "transmitData.h"
#include "common.h"
#include <stdint.h>
#include <opencv2/opencv.hpp>

int main()
{
    TransmitData<unsigned char> testTransmit(CLIENT_IP, WIFI_IMAGE_PORT);

    cv::Mat img = cv::imread("/home/joeyblack/Documents/School/ECE1896/senior_design/network_stuff/rover/src/testSources/testImage.jpg", cv::IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << std::endl;
        return 1;
    }
    std::cout << "Image Size: " << img.total() * img.elemSize() << std::endl;

    while(1)
    {
        testTransmit.sendPayload(img.data, img.total() * img.elemSize());
        std::cout << "Sent image" << std::endl;
    }
}