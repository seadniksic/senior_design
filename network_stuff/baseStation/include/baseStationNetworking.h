#ifndef BASESTATIONNETWORKING_H
#define BASESTATIONNETWORKING_H

#include "receiveData.h"
#include "transmitData.h"
#include "common.h"
#include <memory>

void initializeNetwork(void);
void shutdownNetwork(void);

void getCameraData(void);
void getPointCloudData(void);
void getRoverStatus(void);
void sendRoverCommands(void);
void getCameraPosition(void);

#endif