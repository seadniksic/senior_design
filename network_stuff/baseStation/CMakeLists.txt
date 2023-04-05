cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_FLAGS "-pthread")

project(BaseStationApplication VERSION 1.0.0 DESCRIPTION "The base station application")

include_directories(include/)
include_directories(../networkLibrary/include)
include_directories(../common/)

set(BASE_STATION_CONNECTIONS_SRC
    src/baseStationConnections.cpp
    ../networkLibrary/lib/receiveData.cpp
    ../networkLibrary/lib/transmitData.cpp
    ../networkLibrary/lib/passThroughWire.cpp
)
add_executable(baseStationConnections ${BASE_STATION_CONNECTIONS_SRC})