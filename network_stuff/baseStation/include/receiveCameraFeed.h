#ifndef RECEIVECAMERAFEED_H
#define RECEIVECAMERAFEED_H
#include "receiveData.h"
#include <opencv2/opencv.hpp>

typedef cv::Mat image_t;

void initServer(uint16_t port);
void getImageData(image_t *buffer, size_t bufferSize);
void turnOffServer();

#endif