#include <iostream>
#include "transmitData.h"
#include "common.h"
#include <stdint.h>
#include <opencv2/opencv.hpp>

int main()
{
    TransmitData<unsigned char> testTransmit(CLIENT_IP, WIFI_IMAGE_PORT);

    cv::Mat img = cv::imread("/home/joeyblack/Documents/School/ECE1896/senior_design/network_stuff/rover/src/testSources/testImage.jpg", cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread("/home/joeyblack/Documents/School/ECE1896/senior_design/network_stuff/rover/src/testSources/testImage2.jpg", cv::IMREAD_COLOR);
    cv::Mat img3 = cv::imread("/home/joeyblack/Documents/School/ECE1896/senior_design/network_stuff/rover/src/testSources/testImage_3.jpg", cv::IMREAD_COLOR);


    while(1)
    {
        testTransmit.sendPayload(img.data, img.total() * img.elemSize());
        std::cout << "Sent image" << std::endl;
        sleep(2);
        testTransmit.sendPayload(img2.data, img2.total() * img2.elemSize());
        std::cout << "Sent image" << std::endl;
        sleep(2);
        testTransmit.sendPayload(img3.data, img3.total() * img3.elemSize());
        std::cout << "Sent image" << std::endl;
        sleep(2);
    }
}