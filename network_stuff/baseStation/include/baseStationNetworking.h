#ifndef BASESTATIONNETWORKING_H
#define BASESTATIONNETWORKING_H

#include "receiveData.h"
#include "transmitData.h"
#include "common.h"

void initializeNetwork(void);
void shutdownNetwork(void);

void getCameraData(void);
void getSlamData(void);
void getRoverStatus(void);
void sendRoverCommands(void);

#endif