#ifndef BASESTATIONGUINETWORKING_H
#define BASESTATIONGUINETWORKING_H

#include "receiveData.h"
#include "transmitData.h"
#include "common.h"

void initializeNetwork(void);
void shutdownNetwork(void);

int getCameraData(image_t *buffer, size_t bufferSize);
int getSlamData(slam_t *buffer, size_t bufferSize);
int getRoverStatus(status_t *buffer, size_t bufferSize);
void sendRoverCommands();

#endif