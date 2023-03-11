#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

using namespace cv;
using namespace dnn;

using namespace std;
//using namespace tensorflow;

int main() 
{
    
// Specify the paths for the 2 files
string protoFile = "../models/pose/mpi/pose_deploy_linevec.prototxt";
string weightsFile = "../models/pose/mpi/pose_iter_160000.caffemodel";

// Read the network into Memory
//Net net = readNetFromTensorflow("../models/movenet_multipose_lightning_1/saved_model.pb");
//cout << "Read";

  string videoArgs = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

  //Start capture
  VideoCapture cam(videoArgs, cv::CAP_GSTREAMER);

  if(!cam.isOpened())
    {
       printf("Camera is not opened.\n");
       return -1;
    }

    while(1)
    {

	//Executed for every frame


        Mat frame;
        cam >> frame;
        
        //cv::Size s = frame.size();
	//int frameHeight = s.height;
	//int frameWidth = s.width;
        
        // Specify the input image dimensions
	//int inWidth = 256;
	//int inHeight = 144;

// Prepare the frame to be fed to the network
	//Mat inpBlob = blobFromImage(frame, 1.0 / 255, Size(inWidth, inHeight), Scalar(0, 0, 0), false, false);

// Set the prepared object as the input blob of the network
	//net.setInput(inpBlob);
	
	//cout << "Here";
	
	//Mat output = net.forward();
	
	//cout << "There";
	
	//int H = output.size[2];
	//int W = output.size[3];
	
	//int nPairs = 18;
	///
	///Mat trimmed = output[0][0]
	

// find the position of the body parts
	//vector<Point> points(17);
	//for (int i=0; i < 17; i++) {
	//for (int n=0; n < nPairs; n++)
	//{
// Probability map of corresponding body's part.
		//Mat probMap(H, W, CV_32F, output.ptr(0,n));

		//Point2f p(-1,-1);
		//Point maxLoc;
		//double prob;
		//minMaxLoc(probMap, 0, &prob, 0, &maxLoc);
		//if (prob > .6)
		//{
		//p = maxLoc;
		//p.x *= (float)frameWidth / W ;
		//p.y *= (float)frameHeight / H ;
		
		

		//circle(frame, cv::Point((int)p.x, (int)p.y), 8, Scalar(0,255,255), -1);
		//cv::putText(frame, cv::format("%d", n), cv::Point((int)p.x, (int)p.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
		//}
		//points[n] = p;
	//}
	//}
	
//	for (int n = 0; n < nPairs; n++)
//	{
// lookup 2 connected body/hand parts
//		Point2f partA = points[POSE_PAIRS[n][0]];
//		Point2f partB = points[POSE_PAIRS[n][1]];

//	if (partA.x<=0 || partA.y<=0 || partB.x<=0 || partB.y<=0)
//continue;

//line(frame, partA, partB, Scalar(0,255,255), 8);
//circle(frame, partA, 8, Scalar(0,0,255), -1);
//circle(frame, partB, 8, Scalar(0,0,255), -1);
//}
	
	
        imshow("original",frame);
        if((char)waitKey(30) == 27)
            break;
    }
    
    return 0;
}
