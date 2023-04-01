#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import socket, select
import uart_messages_pb2

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

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('roverStatusPublisher')
        self.roverStatusPublisher = self.create_publisher(Image, 'roverStatusStream', 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(("127.0.0.1", 8091))
        self.socket.listen(1)

        timerPeriod = 1 / 60
        self.create_timer(timerPeriod, self.roverStatusTimerCallBack)
        self.client = None
        self.bridge = CvBridge()
    
    def roverStatusTimerCallBack(self):
        try:
            if self.client is None:
                readSocket, writeSocket, errorSocket = select.select([self.socket], [], [], 0)
                if len(readSocket) == 0:
                    return
                self.client, addr = self.socket.accept()
            readSocket, writeSocket, errorSocket = select.select([self.client], [], [], 0)
            if len(readSocket) == 0:
                return
            size = self.client.recv(8)
            size = int.from_bytes(size, 'little')
            if size <= 0:
                return
            msg = self.client.recv(size)
            while len(msg) < size:
                msg += self.client.recv(size - len(msg))
            
            response = uart_messages_pb2.GUI_Data()
            try:
                response.ParseFromString(msg)
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
            self.roverStatusPublisher.publish(rosMsg)

        except Exception as e:
            print("Rover Status Publisher ran into " + str(e))
    
    def shutdownNode(self):
        if self.socket is not None:
            self.socket.close()
        if self.client is not None:
            self.client.close()

if __name__ == '__main__':
    try:
        rclpy.init()
        roverStatusNode = RoverStatusPublisher()
        rclpy.spin(roverStatusNode)
        roverStatusNode.shutdownNode()
        roverStatusNode.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

    finally:
        roverStatusNode.shutdownNode()