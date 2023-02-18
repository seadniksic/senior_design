cmake_minimum_required (VERSION 3.9)

project(BaseStationApplication VERSION 1.0.0 DESCRIPTION "The base station application")

include_directories(../${CMAKE_CURRENT_SOURCE_DIR}/networkLibrary/include)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(include)

set(SRC_FILES
    src/baseStationApp.cpp
    lib/receiveCameraFeed.cpp
)

set(TEST_FILES
    src/testTransmit.cpp
)

add_executable(baseStationApp ${SRC_FILES})
add_executable(testTransmitBaseStation ${TEST_FILES})

target_link_libraries(baseStationApp PUBLIC NetworkLibrary ${OpenCV_LIBS})
target_link_libraries(testTransmitBaseStation PUBLIC NetworkLibrary)