#ifndef ROVERNETWORKING_H
#define ROVERNETWORKING_H

#include "receiveData.h"
#include "transmitData.h"
#include "common.h"
#include <memory>

void initializeNetwork(void);
void shutdownNetwork(void);

void sendCameraData(void);
void sendPointCloudData(void);
void sendRoverStatus(void);
void getRoverCommands(void);
void sendCameraPosition(void);

#endif