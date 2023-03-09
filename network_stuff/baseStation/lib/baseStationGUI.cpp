#include "baseStationGUI.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    //Create a QTimer is runs at 30 FPS to handle updating the GUI
    updateTimer.setInterval(33);
    connect(&updateTimer, &QTimer::timeout, this, &MainWindow::timerUpdate);
    updateTimer.start();

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    QTabWidget *tabWidget = new QTabWidget(centralWidget);

    QWidget *cameraTab = new QWidget(tabWidget);
    QVBoxLayout *cameraTabLayout = new QVBoxLayout(cameraTab);
    cameraTabLabel = new QLabel(cameraTab);
    cameraTabLayout->addWidget(cameraTabLabel);
    tabWidget->addTab(cameraTab, "Camera Feed");

    cameraFeed = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    cameraTabLabel->setPixmap(QPixmap::fromImage(createQImage(*cameraFeed)));

    QWidget *slamTab = new QWidget(tabWidget);
    QVBoxLayout *slamTabLayout = new QVBoxLayout(slamTab);
    slamTabLabel = new QLabel(slamTab);
    slamTabLayout->addWidget(slamTabLabel);
    tabWidget->addTab(slamTab, "Slam Feed");

    slamFeed = new cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    slamTabLabel->setPixmap(QPixmap::fromImage(createQImage(*slamFeed)));

    mainLayout->addWidget(tabWidget);
    centralWidget->setLayout(mainLayout);

    initializeNetwork();
}

MainWindow::~MainWindow()
{
    delete cameraFeed;
    delete slamFeed;
    delete cameraTabLabel;
    delete slamTabLabel;

    shutdownNetwork();
}

void MainWindow::timerUpdate()
{
    if(getCameraData(cameraFeed->data, cameraFeed->total() * cameraFeed->elemSize()) > 0)
        cameraTabLabel->setPixmap(QPixmap::fromImage(createQImage(*cameraFeed)));
    if(getSlamData(slamFeed->data, slamFeed->total() * slamFeed->elemSize()) > 0)
        slamTabLabel->setPixmap(QPixmap::fromImage(createQImage(*slamFeed)));
}

QImage MainWindow::createQImage(cv::Mat input)
{
    cv::Mat tempMatrix;
    cvtColor(input, tempMatrix, cv::COLOR_RGB2BGR);
    QImage newImage((const uchar *) tempMatrix.data, tempMatrix.cols, tempMatrix.rows, tempMatrix.step, QImage::Format_RGB888);
    newImage.bits();
    return newImage;
}
