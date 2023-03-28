#!/usr/bin/env python
import rospy
import csv
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped


curr_pose = None
actual_pose = None
avg = 0
trials = 0

def callback(data):
    global curr_pose
    global actual_pose
    global avg
    global trials
    if curr_pose and actual_pose:
        p1 = np.array([curr_pose.pose.position.x, curr_pose.pose.position.y])#, curr_pose.pose.position.z])
        
        p2 = np.array([-(actual_pose.point.x-4.688319), actual_pose.point.y + 1.786938])#, actual_pose.point.z-0.783338])
        err = np.sqrt(np.sum((p1-p2)**2, axis=0))
        avg += err
        trials+=1
        #print(err, p1, p2)
        print(f"Avg Err: {avg/trials} meters")
        curr_pose = actual_pose = None
    curr_pose = data

def callback2(data):
    global actual_pose
    actual_pose = data
   
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/orbslam3/odom", PoseStamped, callback)
    rospy.Subscriber("/leica/position", PointStamped, callback2)
    rospy.spin()

if __name__ == '__main__':
    print("run")
    listener()
