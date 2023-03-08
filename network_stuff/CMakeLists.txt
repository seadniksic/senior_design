cmake_minimum_required(VERSION 3.9)
set(CMAKE_AUTOMOC ON)

project(BaseStationApplication VERSION 1.0.0 DESCRIPTION "The base station application")

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Qt5 COMPONENTS Widgets REQUIRED)
include_directories(${Qt5_INCLUDE_DIRS})

include_directories(baseStation/include)
include_directories(networkLibrary/include)
include_directories(common/)

set(SRC_FILES
    baseStation/src/baseStationApp.cpp
    baseStation/lib/receiveCameraFeed.cpp
    networkLibrary/lib/receiveData.cpp
    networkLibrary/lib/transmitData.cpp
    baseStation/lib/baseStationGUI.cpp
)

set(BASE_STATION_TEST
    baseStation/src/testTransmit.cpp
    networkLibrary/lib/transmitData.cpp
)

set(ROVER_TEST
    rover/src/testReceive.cpp
    networkLibrary/lib/receiveData.cpp
)

set(ROVER_IMAGE_TEST
    rover/src/testImageTransmit.cpp
    networkLibrary/lib/transmitData.cpp
)

add_executable(baseStationApp ${SRC_FILES})
add_executable(baseStationTest ${BASE_STATION_TEST})
add_executable(roverTest ${ROVER_TEST})
add_executable(roverImageTest ${ROVER_IMAGE_TEST})

target_link_libraries(baseStationApp PUBLIC ${OpenCV_LIBS} Qt5::Widgets)
target_link_libraries(roverImageTest PUBLIC ${OpenCV_LIBS})