Welcome to the world of ROS publishing!

None of the code in this file is correct but it works.  The 4 files that are of concern are the cameraPositionPublisher.py,
imageFeedPublisherNode.py, pointCloudPublisherNode.py, and roverStatusPublisher.py.  All the other files are garbage and can 
be ignored.  

imageFeedPublisherNode.py creates a node that takes in a byte array over a socket and then publishes an image to 'cameraPositionStream'.

pointCloudPublisherNode.py creates a node that takes in a byte array over a socket and then publishes a point cloud to 'pointCloudStream'.

cameraPositionPublisher.py creates a node that takes in a byte array over a socket and then publishes a Point to 'cameraPositionStream'.

roverStatusPublisher.py creates a node that takes in a byte array over a socket and then publishes a string to 'roverStatusStream'.

The scripts can be ran on their own or the class can be included in another script.  Need to figure out how to get all of them in a 
ROS launch script.