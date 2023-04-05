#!/usr/bin/env python

import rospy, cv2, uart_messages_pb2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import UInt8MultiArray

def createString(msg):
    output = "CPU Temperature: " + str(msg.cpu_temp) + "\n"
    output += "System Calibration Status: " + str((msg.calib_status & 192) >> 6) + "\n"
    output += "Gyroscope Calibration Status: " + str((msg.calib_status & 30) >> 4) + "\n"
    output += "Accelerometer Calibration Status: " + str((msg.calib_status & 12) >> 2) + "\n"
    output += "Magnometer Calibration Status: " + str((msg.calib_status & 3)) + "\n"
    output += "Calibration Status: " + str(msg.calib_status) + "\n"
    output  += "HTS Temperature: " + str(msg.hts_temp) + "\n"
    output += "HTS Humidity: " + str(msg.hts_humidity) + "\n"
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
        self.publisher_ = rospy.Publisher('roverStatusStream', MarkerArray, queue_size=10)
        self.subscriber_ = rospy.Subscriber('roverStatusNetworkNodePublisher', UInt8MultiArray, self.callback)

        rospy.spin()
    
    def callback(self, msg):
        try:
            response = uart_messages_pb2.GUI_Data()
            try:
                response.ParseFromString(msg.data)
            except Exception as e:
                print("Fucking idiot ran into " + str(e))
            newOutput = createString(response)

            marker_array = MarkerArray()
            text_marker = Marker()
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = rospy.Time.now()
            text_marker.pose.position.x = 0.5 # Set the x-coordinate to move the text to the left
            text_marker.pose.position.y = 0.5 # Set the y-coordinate to move the text to the top
            text_marker.pose.position.z = 1.5
            text_marker.scale.z = 0.1 # Set a smaller font size
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = newOutput
            marker_array.markers.append(text_marker)
            
            self.publisher_.publish(marker_array)

        except Exception as e:
            print("Rover Status Publisher ran into: " + str(e))

if __name__ == '__main__':
    try:
        roverStatusNode = RoverStatusPublisher()

        rospy.spin()
    except KeyboardInterrupt:
        pass
