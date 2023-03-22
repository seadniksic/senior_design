#ifndef BASESTATIONGUI_CPP
#define BASESTATIONGUI_CPP

#include "baseStationGUI.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    //Create a QTimer is runs at 30 FPS to handle updating the GUI
    updateTimer.setInterval(15);
    connect(&updateTimer, &QTimer::timeout, this, &MainWindow::timerUpdate);
    updateTimer.start();

    //Create Main layout as well as the tab widget
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);
    QTabWidget *tabWidget = new QTabWidget(centralWidget);

    //Handle the camera feed tab
    QWidget *cameraTab = new QWidget(tabWidget);
    QHBoxLayout *cameraTabLayout = new QHBoxLayout(cameraTab);

    cameraTabLabel = new QLabel(cameraTab);
    cameraTabLayout->addWidget(cameraTabLabel);
    cameraFeed = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    cameraTabLabel->setPixmap(QPixmap::fromImage(createQImage(*cameraFeed)));

    tabWidget->addTab(cameraTab, "Camera Feed");

    //Handle the slam data tab
    QWidget *slamTab = new QWidget(tabWidget);
    QHBoxLayout *slamTabLayout = new QHBoxLayout(slamTab);
    slamTabLabel = new QLabel(slamTab);
    slamTabLayout->addWidget(slamTabLabel);
    slamFeed = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0, 0, 0));
    slamTabLabel->setPixmap(QPixmap::fromImage(createQImage(*slamFeed)));

    tabWidget->addTab(slamTab, "Slam Feed");

    roverStatus = new QLabel();
    roverStatus->setText("Current Temperature: " + QString::number(0));
    currentRoverData = new status_t;

    //Set main layout and main widget
    mainLayout->addWidget(tabWidget, 8);
    mainLayout->addWidget(roverStatus, 2);
    centralWidget->setLayout(mainLayout);
}

MainWindow::~MainWindow()
{
    delete cameraFeed;
    delete slamFeed;
    delete cameraTabLabel;
    delete slamTabLabel;
}

void MainWindow::timerUpdate()
{
    if(getCameraData(cameraFeed->data, cameraFeed->total() * cameraFeed->elemSize()) > 0)
    {
        std::vector<uchar> data(cameraFeed->data, cameraFeed->data + cameraFeed->total() * cameraFeed->elemSize());
        cv::imdecode(cv::Mat(data), cv::IMREAD_COLOR, cameraFeed);
        cameraTabLabel->setPixmap(QPixmap::fromImage(createQImage(*cameraFeed)));
    }
    if(getSlamData(slamFeed->data, slamFeed->total() * slamFeed->elemSize()) > 0)
    {
        slamTabLabel->setPixmap(QPixmap::fromImage(createQImage(*slamFeed)));
    }
    if(getRoverStatus(currentRoverData, sizeof(status_t)) > 0)
        roverStatus->setText("Current Temperature: " + QString::number(currentRoverData->battery));
}

QImage MainWindow::createQImage(cv::Mat input)
{
    cv::Mat tempMatrix;
    cvtColor(input, tempMatrix, cv::COLOR_RGB2BGR);
    QImage newImage((const uchar *) tempMatrix.data, tempMatrix.cols, tempMatrix.rows, tempMatrix.step, QImage::Format_RGB888);
    newImage.bits();
    return newImage;
}

#endif