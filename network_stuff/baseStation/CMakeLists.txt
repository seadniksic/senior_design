cmake_minimum_required (VERSION 3.9)

project(BaseStationApplication VERSION 1.0.0 DESCRIPTION "The base station application")

include_directories(../${CMAKE_CURRENT_SOURCE_DIR}/NetworkLibrary/include)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(include)

set(SRC_FILES
    src/baseStationApp.cpp
    lib/receiveCameraFeed.cpp
)

add_executable(baseStation ${SRC_FILES})

target_link_libraries(baseStation PUBLIC NetworkLibrary)