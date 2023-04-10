#!/usr/bin/env python

import rospy, pickle
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import UInt8MultiArray, Header
from tf.msg import tfMessage

class CameraPositionPublisher:
    def __init__(self):
        rospy.init_node('cameraPositionPublisher')
        self.publisher_ = rospy.Publisher('/tf', tfMessage, queue_size=10)
        self.subscriber_ = rospy.Subscriber('cameraPositionNetworkNodePublisher', UInt8MultiArray, self.callback)

        rospy.spin()
    
    def callback(self, msg):
        try:
            newHeader = Header()
            newHeader.stamp = rospy.Time.now()
            newHeader.frame_id = "map"
            
            pointData = pickle.loads(msg.data)
            # newPoseStamp = PoseStamped()
            # newPoseStamp.header = newHeader
            # newPoseStamp.pose = pointData

            # # print(type(newPoseStamp))
            newTf = TransformStamped()
            newTf.header = newHeader
            newTf.child_frame_id = "rover"
            newTf.transform.translation = pointData.position
            newTf.transform.rotation = pointData.orientation

            tfm = tfMessage([newTf])
            

            self.publisher_.publish(tfm)

        except Exception as e:
            print("Camera Position Publisher ran into: " + str(e))

if __name__ == '__main__':
    try:
        cameraPositionNode = CameraPositionPublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass