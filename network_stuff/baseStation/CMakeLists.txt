cmake_minimum_required(VERSION 3.9)
set(CMAKE_AUTOMOC ON)

project(BaseStationApplication VERSION 1.0.0 DESCRIPTION "The base station application")

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Qt5 COMPONENTS Widgets REQUIRED)
include_directories(${Qt5_INCLUDE_DIRS})

include_directories(include/)
include_directories(../networkLibrary/include)
include_directories(../common/)

set(GUI_SRC_FILES
    src/baseStationApp.cpp
    lib/baseStationGUINetworking.cpp
    ../networkLibrary/lib/receiveData.cpp
    ../networkLibrary/lib/transmitData.cpp
    lib/baseStationGUI.cpp
)

set(BASE_STATION_TEST_TRANSMIT
    src/testTransmit.cpp
    ../networkLibrary/lib/transmitData.cpp
)

set(BASE_STATION_CONNECTIONS_SRC
    src/baseStationConnections.cpp
    lib/baseStationNetworking.cpp
    ../networkLibrary/lib/receiveData.cpp
    ../networkLibrary/lib/transmitData.cpp
)

add_executable(baseStationApp ${GUI_SRC_FILES})
add_executable(baseStationTest ${BASE_STATION_TEST_TRANSMIT})
add_executable(baseStationConnections ${BASE_STATION_CONNECTIONS_SRC})
target_link_libraries(baseStationApp PUBLIC ${OpenCV_LIBS} Qt5::Widgets)