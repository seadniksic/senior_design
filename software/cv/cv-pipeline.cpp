#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/public/session.h>

using namespace cv;
using namespace std;
using namespace tensorflow;

int main() 
{
    
  ...
  // set up your input paths
  const string pathToGraph = "models/saved_model.pb"
  //const string checkpointPath = "models/my-model";
  ...

  auto session = NewSession(SessionOptions());
  if (session == nullptr) {
    throw runtime_error("Could not create Tensorflow session.");
  }

  Status status;

  // Read in the protobuf graph we exported
  MetaGraphDef graph_def;
  status = ReadBinaryProto(Env::Default(), pathToGraph, &graph_def);
  if (!status.ok()) {
    throw runtime_error("Error reading graph definition from " + pathToGraph + ": " + status.ToString());
  }

  // Add the graph to the session
  status = session->Create(graph_def.graph_def());
  if (!status.ok()) {
    throw runtime_error("Error creating graph: " + status.ToString());
  }

  // Read weights from the saved checkpoint
  Tensor checkpointPathTensor(DT_STRING, TensorShape());
  checkpointPathTensor.scalar<std::string>()() = checkpointPath;
  status = session->Run(
        {{ graph_def.saver_def().filename_tensor_name(), checkpointPathTensor },},
        {},
        {graph_def.saver_def().restore_op_name()},
        nullptr);
  if (!status.ok()) {
    throw runtime_error("Error loading checkpoint from " + checkpointPath + ": " + status.ToString());
  }

  // and run the inference to your liking
  auto feedDict = ...
  auto outputOps = ...
  std::vector<tensorflow::Tensor> outputTensors;
  status = session->Run(feedDict, outputOps, {}, &outputTensors);

  string videoArgs = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

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
        imshow("original",frame);
        if((char)waitKey(30) == 27)
            break;
    }
    
    return 0;
}
