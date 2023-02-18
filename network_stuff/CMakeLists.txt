cmake_minimum_required(VERSION 3.9)
# set(CMAKE_VERBOSE_MAKEFILE ON)

add_subdirectory(networkLibrary)
add_subdirectory(baseStation)
add_subdirectory(rover)

# cmake_minimum_required (VERSION 3.9)

# project(BaseStationApplication VERSION 1.0.0 DESCRIPTION "The base station application")

# find_package(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} )

# include_directories(baseStation/include)
# include_directories(networkLibrary/include)

# set(SRC_FILES
#     baseStation/src/baseStationApp.cpp
#     baseStation/lib/receiveCameraFeed.cpp
#     networkLibrary/lib/receiveData.cpp
#     networkLibrary/lib/transmitData.cpp
# )

# add_executable(baseStationApp ${SRC_FILES})

# target_link_libraries(baseStationApp PUBLIC ${OpenCV_LIBS})