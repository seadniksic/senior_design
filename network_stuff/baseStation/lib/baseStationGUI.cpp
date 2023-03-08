#include "baseStationGUI.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    //Create a QTimer is runs at 30 FPS to handle updating the GUI
    updateTimer.setInterval(2000);
    connect(&updateTimer, &QTimer::timeout, this, &MainWindow::timerUpdate);
    updateTimer.start();

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    QTabWidget *tabWidget = new QTabWidget(centralWidget);

    QWidget *cameraTab = new QWidget(tabWidget);
    QVBoxLayout *cameraTabLayout = new QVBoxLayout(cameraTab);
    QLabel *cameraTabLabel = new QLabel(cameraTab);
    cameraTabLayout->addWidget(cameraTabLabel);
    tabWidget->addTab(cameraTab, "Camera Feed");

    cameraFeed = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    QImage cameraFeedImage(cameraFeed->data, cameraFeed->cols, cameraFeed->rows, QImage::Format_RGB888);
    cameraTabLabel->setPixmap(QPixmap::fromImage(cameraFeedImage));

    QWidget *slamTab = new QWidget(tabWidget);
    QVBoxLayout *slamTabLayout = new QVBoxLayout(slamTab);
    QLabel *slamTabLabel = new QLabel(slamTab);
    slamTabLayout->addWidget(slamTabLabel);
    tabWidget->addTab(slamTab, "Slam Feed");

    slamFeed = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    QImage slamFeedImage(slamFeed->data, slamFeed->cols, slamFeed->rows, QImage::Format_RGB888);
    slamTabLabel->setPixmap(QPixmap::fromImage(slamFeedImage));

    mainLayout->addWidget(tabWidget);
    centralWidget->setLayout(mainLayout);

    initServer(WIFI_IMAGE_PORT);
}

MainWindow::~MainWindow()
{
    delete cameraFeed;
    delete slamFeed;

    turnOffServer();
}

void MainWindow::timerUpdate()
{
    if(checkAvailableData())
        getImageData(cameraFeed->data, cameraFeed->total() * cameraFeed->elemSize());
}
