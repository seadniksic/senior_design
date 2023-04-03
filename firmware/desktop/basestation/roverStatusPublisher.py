#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket, select
import uart_messages_pb2

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('roverStatusPublisher')
        self.roverStatusPublisher = self.create_publisher(String, 'roverStatusStream', 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(("127.0.0.1", 8091))
        self.socket.listen(1)

        timerPeriod = 1 / 60
        self.create_timer(timerPeriod, self.roverStatusTimerCallBack)
        self.client = None
    
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
            
            calib_stat_str = bin(response.calib_status)
            local_stat = bin(response.loco_status)
            print(f"cpu_temp={response.cpu_temp}, calib_stat={calib_stat_str}, servo_pan={response.curr_servo_pan}, servo_tilt={response.curr_servo_tilt}, servo_pan_home={response.home_servo_pan}, servo_tilt_home={response.home_servo_tilt}, loco_status={local_stat}")
            print(response.cpu_temp)
            self.roverStatusPublisher.publish(msg)

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