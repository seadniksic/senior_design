cmake_minimum_required(VERSION 3.9)

project(NetworkLibrary VERSION 1.0.0 DESCRIPTION "Network library with simple send and receive")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(NetworkLibrary SHARED
    lib/receiveData.cpp
    lib/transmitData.cpp
)