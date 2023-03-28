#!/usr/bin/python3

import time
from time import sleep
import uart_messages_pb2
import evdev
import sys
from evdev import categorize, ecodes
from timeit import default_timer as timer
import random
import socket, pickle, sys


############################
####      GLOBALS     ######
############################

class g:
    update_rate = 20 # in hz
    sleep_time = 1/ update_rate
    joy_name = "Logitech Gamepad F710"
    test_input = False
    sync_byte = 100 #0x64


############################
####     JOYSTICK     ######
############################

class Joystick:
    def __init__(self):
        self.dev = None
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            print(device.path, device.name, device.phys)
            if(device.name == g.joy_name):
                self.dev = device

        # mapping codes to the Buttons enum in protobuf
        self.CODE_LUT = {
            304 : 1,
            305 : 2,
            307 : 4,
            308 : 8,
            315 : 16,
            314 : 32,

            "UP" : 64,
            "DOWN" : 128,
            "LEFT" : 256,
            "RIGHT" : 512,
            318 : 1024,
            317 : 2048,
            311 : 4096,
            310 : 8192
        }


    def print_dev_capabilities(self, verbose_ouput : bool):
        if self.dev != None:
            print("---------------")
            print("Printing Device Capabilities")
            for item in self.dev.capabilities(verbose=verbose_ouput).items():
                print("")
                print(item) 
            print("---------------")

    # function reads and prints joy inputs indefinitely
    def test_inputs(self):
        if self.dev != None:
            for event in self.dev.read_loop():

                # all buttons (including the joystick switch) besides mode, vibration, and D pad.
                # and logitech button which idk what that does.
                if event.type == ecodes.EV_KEY:
                    # print(categorize(event))
                    print(event.code, event.value)

                # mode looks like it switches D -pad with Left joystick in terms of the values that it reads.
                # but on these event is dpad rjoy and ljoy
                if event.type == ecodes.EV_ABS:
                    # print(categorize(event))
                    print(event)

                # none of the buttons are on here
                if event.type == ecodes.EV_REL:
                    print(categorize(event))


############################
#### GLOBAL FUNCTIONS ######
############################

def handle_args():
    # handle args
    if len(sys.argv) > 1: 
        first_arg = sys.argv[1].lower()
        if first_arg == "test":
            g.test_input = True
            print("In testing mode, will just print controller inputs (no data will be sent)")
        
    if not g.test_input:
        print("In normal operation mode.")

    sleep(1.5)

############################
####     MAIN CODE    ######
############################

if __name__ == "__main__":

    # get joystick
    joystick = Joystick()

    if joystick.dev is None:
        print("JOYSTICK NOT FOUND!! EXITING...")
        exit()

    # print device capabilities
    joystick.print_dev_capabilities(False)
    
    # handle script args and update g class with values
    handle_args()

    # If in testing mode, call test. function will never return
    if g.test_input:
        joystick.test_inputs()


    # joystick data
    btn_data = 0
    ljoy_x_data = 0
    ljoy_y_data = 0
    rjoy_x_data = 0
    rjoy_y_data = 0
    tr_data = 0
    tl_data = 0

    # timers
    # end minus start gives time in sec
    start_time = timer()
    end_time = None

    # networking
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = socket.gethostname()
    port = 8088 
    sock.connect((host, port))

    while True: 
        event = joystick.dev.read_one()
        if event != None:
            if event.type == ecodes.EV_KEY:
                # this is not the best way to do it but yolo for now
                # bit arrays might be nice
                bit_mask = joystick.CODE_LUT[event.code]
                if event.value:
                    btn_data = btn_data | bit_mask
                else: 
                    btn_data = btn_data & ~bit_mask
            if event.type == ecodes.EV_ABS:
                # dpad up down is 17
                # dpad left right is 16
                if event.code == 16:
                    # dpad right is pressed
                    if event.value == 1:
                        btn_data = btn_data | joystick.CODE_LUT["RIGHT"]
                        btn_data = btn_data & ~joystick.CODE_LUT["LEFT"]
                    elif event.value == -1:
                        btn_data = btn_data & ~joystick.CODE_LUT["RIGHT"]
                        btn_data = btn_data | joystick.CODE_LUT["LEFT"]
                    else:
                        btn_data = btn_data & ~joystick.CODE_LUT["RIGHT"]
                        btn_data = btn_data & ~joystick.CODE_LUT["LEFT"]
                elif event.code == 17:
                    # dpad right is pressed
                    if event.value == 1:
                        btn_data = btn_data | joystick.CODE_LUT["DOWN"]
                        btn_data = btn_data & ~joystick.CODE_LUT["UP"]
                    elif event.value == -1:
                        btn_data = btn_data & ~joystick.CODE_LUT["DOWN"]
                        btn_data = btn_data | joystick.CODE_LUT["UP"]
                    else:
                        btn_data = btn_data & ~joystick.CODE_LUT["DOWN"]
                        btn_data = btn_data & ~joystick.CODE_LUT["UP"]
                
                # rjoy y
                elif event.code == 4:
                    rjoy_y_data = event.value
                
                #rjoy x
                elif event.code == 3:
                    rjoy_x_data = event.value
                
                #ljoy y
                elif event.code == 1:
                    ljoy_y_data = event.value
                
                #ljoy x
                elif event.code == 0:
                    ljoy_x_data = event.value
                
                # RT
                elif event.code == 5:
                    tr_data = event.value
                
                # LT
                elif event.code == 2:
                    tl_data = event.value
                
        
        #check if its time to send data
        end_time = timer()
        if (end_time - start_time) > g.sleep_time:
            
            start_time = end_time
            
            # store joystick message
            proto_msg = uart_messages_pb2.Joystick_Input()

            # create byte array for the data to send
            sendData = bytearray()

            # store the sync byte
            # length is the number of bytes it will use when it converts count
            sendData.extend(g.sync_byte.to_bytes(length=1, byteorder='little', signed=False))

            # prepare the proto message to send
            proto_msg.button = btn_data
            proto_msg.LJOY_X = ljoy_x_data
            proto_msg.LJOY_Y = ljoy_y_data            
            proto_msg.RJOY_X = rjoy_x_data
            proto_msg.RJOY_Y = rjoy_y_data
            proto_msg.TR = tr_data
            proto_msg.TL = tl_data


            # serialize the data
            proto_msg_serialized = proto_msg.SerializeToString()

            # store the length of the message and add it to be sent
            msg_len = len(proto_msg_serialized)
            
            # print("proto msg", proto_msg_serialized)
            # print("msg length",msg_len)
            sendData.extend(msg_len.to_bytes(length=1, byteorder='little', signed = False))
            
            # add the proto data
            sendData.extend(proto_msg_serialized)
            print(sendData)
            
            # send the data over wifi
            messageSize = int(len(sendData)).to_bytes(8, byteorder='little', signed=False)
            sock.sendall(messageSize)
            sock.sendall(sendData)