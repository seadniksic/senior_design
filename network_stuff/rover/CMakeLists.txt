cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_FLAGS "-pthread")

project(RoverApplication VERSION 1.0.0 DESCRIPTION "The rover application")

# find_package(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(../networkLibrary/include)
include_directories(include/)
include_directories(../common/)

set(ROVER_TRANSMIT_FILES
    src/roverConnections.cpp
    lib/roverNetworking.cpp
    ../networkLibrary/lib/receiveData.cpp
    ../networkLibrary/lib/transmitData.cpp
)

# set(ROVER_STRING_TEST
#     src/testReceive.cpp
#     ../networkLibrary/lib/receiveData.cpp
# )

# set(ROVER_IMAGE_TEST
#     src/testImageTransmit.cpp
#     ../networkLibrary/lib/transmitData.cpp
# )

# add_executable(roverStringTest ${ROVER_STRING_TEST})
# add_executable(roverImageTest ${ROVER_IMAGE_TEST})
add_executable(roverConnections ${ROVER_TRANSMIT_FILES})

# target_link_libraries(roverImageTest PUBLIC ${OpenCV_LIBS})
# target_link_libraries(roverConnections PUBLIC ${OpenCV_LIBS})