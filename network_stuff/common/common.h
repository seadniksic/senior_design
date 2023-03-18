#ifndef COMMON_H
#define COMMON_H

#define LOCAL_IP "127.0.0.1"
#define HOST_IP "10.42.0.1"
#define CLIENT_IP "10.42.0.12"

#define WIFI_IMAGE_PORT 8080
#define WIFI_SLAM_PORT 8081
#define WIFI_ROVER_COMMANDS_PORT 8082
#define WIFI_ROVER_STATUS_PORT 8083

#define JETSON_IMAGE_PORT 8084
#define JETSON_SLAM_PORT 8085
#define JETSON_STATUS_PORT 8086
#define JETSON_COMMANDS_PORT 8087

#define BASE_STATION_COMMANDS_PORT 8088

#define IMAGE_WIDTH 720
#define IMAGE_HEIGHT 480

#define MAX_PACKET_SIZE 1400

struct roverCommands{
    uint8_t data[48];
};

struct roverStatus{
    uint8_t battery;
};

typedef unsigned char image_t;
typedef unsigned char slam_t;
typedef struct roverCommands commands_t;
typedef struct roverStatus status_t;

#endif