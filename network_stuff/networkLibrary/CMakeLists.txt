cmake_minimum_required(VERSION 3.9)

project(NetworkLibrary VERSION 1.0.0 DESCRIPTION "Network library with simple send and receive")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(NetworkLibrary SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/receiveData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/transmitData.cpp
)

target_include_directories(NetworkLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(NetworkLibrary PUBLIC ${OpenCV_INCLUDE_DIRS})

# set(SRC_FILES
#     src/testNetworkClass.cpp
# )

# add_executable(networkTest ${SRC_FILES})
# target_link_libraries(networkTest PUBLIC NetworkLibrary)