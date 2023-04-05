#!/usr/bin/env python

import rospy, cv2, uart_messages_pb2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
import numpy as np

def createString(msg):
    output = "CPU Temperature: " + str(msg.cpu_temp) + "\n"
    output += "Calibration Status: " + str(msg.calib_status) + "\n"
    output  += "HTS Temperature: " + str(msg.hts_temp) + "\n"
    output += "HTS Humidity: " + str(msg.hts_humidity) + "\n"
    output += "Battery Temperature: " + str(msg.batt_temp) + "\n"
    output += "Current Servo Pan: " + str(msg.curr_servo_pan) + "\n"
    output += "Current Servo Tilt: " + str(msg.home_servo_pan) + "\n"
    output += "Home Servo Pan: " + str(msg.home_servo_pan) + "\n"
    output += "Home Servo Tilt: " + str(msg.home_servo_tilt) + "\n"
    output += "Locomotion Status: " + str(msg.loco_status) + "\n"
    return output

class RoverStatusPublisher:
    def __init__(self):
        rospy.init_node('roverStatusPublisher')
        self.publisher_ = rospy.Publisher('roverStatusStream', Image, queue_size=10)
        self.subscriber_ = rospy.Subscriber('roverStatusNetworkNodePublisher', UInt8MultiArray, self.callback)
        self.bridge = CvBridge()
        rospy.spin()
    
    def callback(self, msg):
        try:
            response = uart_messages_pb2.GUI_Data()
            try:
                response.ParseFromString(msg.data)
            except Exception as e:
                print("Fucking idiot ran into " + str(e))
            newOutput = createString(response)
            display = np.zeros((500, 500, 3), np.uint8)
            display.fill(255)

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            color = (0, 0, 0)
            thickness = 2

            text_size, _ = cv2.getTextSize(newOutput, font, font_scale, thickness)
            text_x = (display.shape[1] - text_size[0]) // 2
            text_y = (display.shape[0] + text_size[1]) // 2
            cv2.putText(display, newOutput, (text_x, text_y), font, font_scale, color, thickness)

            rosMsg = self.bridge.cv2_to_imgmsg(display, encoding="passthrough")
            self.publisher_.publish(rosMsg)

        except Exception as e:
            print("Rover Status Publisher ran into: " + str(e))

if __name__ == '__main__':
    try:
        roverStatusNode = RoverStatusPublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass
