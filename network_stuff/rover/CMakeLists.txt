cmake_minimum_required (VERSION 3.9)

project(RoverApplication VERSION 1.0.0 DESCRIPTION "The rover application")

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(TEST_FILES
    src/testReceive.cpp
)

add_executable(testReceiveRover ${TEST_FILES})

target_link_libraries(testReceiveRover PUBLIC NetworkLibrary)