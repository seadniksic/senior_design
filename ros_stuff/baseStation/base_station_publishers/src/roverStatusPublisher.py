#!/usr/bin/env python

import rospy, cv2, uart_messages_pb2
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import UInt8MultiArray, ColorRGBA

def createString(msg):
    output = "CPU Temperature: " + str(msg.cpu_temp) + "\n"
    output += "System Calibration Status: " + str((msg.calib_status & 192) >> 6) + "\n"
    output += "Gyroscope Calibration Status: " + str((msg.calib_status & 30) >> 4) + "\n"
    output += "Accelerometer Calibration Status: " + str((msg.calib_status & 12) >> 2) + "\n"
    output += "Magnometer Calibration Status: " + str((msg.calib_status & 3)) + "\n"
    output  += "HTS Temperature: " + str(msg.hts_temp / 1000) + "\n"
    output += "HTS Humidity: " + str(msg.hts_humidity / 1000) + "\n"
    output += "Battery Temperature: " + str(msg.batt_temp) + "\n"
    output += "Current Servo Pan: " + str(msg.curr_servo_pan) + "\n"
    output += "Current Servo Tilt: " + str(msg.home_servo_pan) + "\n"
    output += "Home Servo Pan: " + str(msg.home_servo_pan) + "\n"
    output += "Home Servo Tilt: " + str(msg.home_servo_tilt) + "\n"
    output += "Disabled Driving Status: " + str(msg.loco_status & 1) + "\n"
    output += "Diagnonal Driving Status: " + str((msg.loco_status >> 1) & 1) + "\n"
    return output

class RoverStatusPublisher:
    def __init__(self):
        rospy.init_node('roverStatusPublisher')
        self.publisher_ = rospy.Publisher('roverStatusStream', OverlayText, queue_size=10)
        self.subscriber_ = rospy.Subscriber('roverStatusNetworkNodePublisher', UInt8MultiArray, self.callback)

        rospy.spin()
    
    def callback(self, msg):
        try:
            response = uart_messages_pb2.GUI_Data()
            try:
                response.ParseFromString(msg.data)
                print(response)
            except Exception as e:
                print("Fucking idiot ran into " + str(e))

            newMsg = OverlayText()
            newMsg.text = createString(response)
            newMsg.width = 600
            newMsg.height = 600
            newMsg.left = 10
            newMsg.top = 10
            newMsg.text_size = 10
            newMsg.line_width = 2
            newMsg.font = "DejaVu Sans Mono"
            newMsg.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
            newMsg.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
            
            self.publisher_.publish(newMsg)

        except Exception as e:
            print("Rover Status Publisher ran into: " + str(e))

if __name__ == '__main__':
    try:
        roverStatusNode = RoverStatusPublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass
