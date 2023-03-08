#ifndef BASESTATIONGUI_H
#define BASESTATIONGUI_H

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTabWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QImage>
#include <QTimer>
#include <QObject> 
#include <opencv2/opencv.hpp>
#include "common.h"
#include "receiveCameraFeed.h"

class MainWindow : public QMainWindow {
    private:
        cv::Mat *cameraFeed, *slamFeed;
        QTimer updateTimer;
        QLabel *cameraTabLabel;
    
    private slots:
        void timerUpdate();

    public:
        MainWindow(QWidget *parent = nullptr);
        ~MainWindow();
};

#endif