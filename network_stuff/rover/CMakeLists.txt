cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_FLAGS "-pthread")

project(RoverApplication VERSION 1.0.0 DESCRIPTION "The rover application")

include_directories(../networkLibrary/include)
include_directories(include/)
include_directories(../common/)

set(ROVER_TRANSMIT_FILES
    src/roverConnections.cpp
    ../networkLibrary/lib/receiveData.cpp
    ../networkLibrary/lib/transmitData.cpp
    ../networkLibrary/lib/passThroughWire.cpp
)
add_executable(roverConnections ${ROVER_TRANSMIT_FILES})