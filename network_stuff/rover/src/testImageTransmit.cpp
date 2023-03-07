#include <iostream>
#include "transmitData.h"
#include "common.h"
#include <opencv2/opencv.hpp>

int main()
{
    TransmitData<cv::Mat> testTransmit(CLIENT_IP, WIFI_IMAGE_PORT);

    cv::Mat img = cv::imread("/home/joeyblack/Documents/School/ECE1896/senior_design/network_stuff/rover/src/testSources/testImage.jpg", cv::IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << std::endl;
        return 1;
    }
    while(1)
    {
        testTransmit.sendPayload(&img, img.total());
    }
}