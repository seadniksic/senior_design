#ifndef PASSTHROUGHWIRE_H
#define PASSTHROUGHWIRE_H

#include "receiveData.h"
#include "transmitData.h"
#include "common.h"
#include <memory>

class PassThroughWire{
    private:
        std::unique_ptr<ReceiveData> server;
        std::unique_ptr<TransmitData> client;
        std::unique_ptr<char[]> buffer;
        uint64_t bufferSize;
        int debug;
        std::string name;
    public:
        PassThroughWire(const int receivePort, const int transmitPort, const char *transmitIP, const int bufferSize, std::string name, int debugMode);
        void update(void);
};

#endif