#ifndef ROVERNETWORKING_H
#define ROVERNETWORKING_H

#include "receiveData.h"
#include "transmitData.h"
#include "common.h"

void initializeNetwork(void);
void shutdownNetwork(void);

void sendCameraData(void);
void sendSlamData(void);
void sendRoverStatus(void);
void getRoverCommands(void);

#endif