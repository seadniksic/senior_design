#ifndef BASESTATIONGUI_H
#define BASESTATIONGUI_H

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QImage>
#include <QTimer>
#include <QObject> 
#include <opencv2/opencv.hpp>
#include "common.h"
#include "baseStationGUINetworking.h"

class MainWindow : public QMainWindow {
    private:
        cv::Mat *cameraFeed, *slamFeed;
        QTimer updateTimer;
        QLabel *cameraTabLabel, *slamTabLabel, *roverStatus;
        QImage createQImage(cv::Mat input);
        status_t *currentRoverData;
    
    private slots:
        void timerUpdate();

    public:
        MainWindow(QWidget *parent = nullptr);
        ~MainWindow();
};

#endif