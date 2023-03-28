#include "baseStationGUI.h"
#include <iostream>
#include "common.h"
#include "baseStationGUINetworking.h"
#include <pthread.h>

pthread_t commandsThread;

void* commandsThreadFunction(void* arg);

int main(int argc, char *argv[])
{
    initializeNetwork();

    pthread_create(&commandsThread, NULL, commandsThreadFunction, NULL);
    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.show();
    return app.exec();

    pthread_join(commandsThread, NULL);
    shutdownNetwork();
    return 0;
}

void* commandsThreadFunction(void* arg)
{
    while(1)
    {
        sendRoverCommands();
    }

    return NULL;
}