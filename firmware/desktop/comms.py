#!/usr/bin/python3

import time
import serial
import uart_messages_pb2
import evdev
import sys
from evdev import categorize, ecodes
from timeit import default_timer as timer
import random
import pickle, socket

class g:
    update_rate = 20 # in hz
    sleep_time = 1/ update_rate
    joy_name = "Logitech Gamepad F710"
    test_input = False
    sync_byte = 100 #0x64
    remote_joystick = False #if true, joystick is connected to base station laptop

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


def handle_args():
    # handle args
    first_arg = ""
    second_arg = ""
    if len(sys.argv) == 3:
        first_arg = sys.argv[1].lower()
        second_arg = sys.argv[2].lower()
    elif len(sys.argv) == 2:
        first_arg = sys.argv[1].lower()

    if second_arg == "true":
        g.remote_joystick = True
        print("Joystick inputs will not be read, joystick is connected to base station.")
    if g.remote_joystick == False and first_arg == "true":
        g.test_input = True
        print("In testing mode, will just print controller inputs (no data will be sent)")
        
    if not g.test_input and not g.remote_joystick:
        print("In normal operation mode with controller hooked up to the Jetson")
    elif g.remote_joystick and not g.test_input:
        print("In normal operation mode with controller hooked up to base station laptop")

    time.sleep(1.5)


if __name__ == "__main__":

    # handle script args and update g class with values
    handle_args()

    # attach joystick
    if not g.remote_joystick:
        # get joystick
        joystick = Joystick()

        if joystick.dev is None:
            print("JOYSTICK NOT FOUND!! EXITING...")
            exit()
        joystick.print_dev_capabilities(False)

    # create sp= serial port object
    # arduino defualt is 8 data bits, no parity, and one stop bit
    sp = serial.Serial(
        port="/dev/ttyTHS0",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    time.sleep(1)

    if g.test_input and not g.remote_joystick:
        joystick.test_inputs()
        # will not return from this loop

    # always update the struct
    # but send at 10hz
    # if struct doesnt update, no need to send data!
    # read device inputs

    # H means unsigned short so uint16_t
    # data = struct.pack("H", 0)

    btn_data = 0
    ljoy_x_data = 0
    ljoy_y_data = 0
    rjoy_x_data = 0
    rjoy_y_data = 0
    tr_data = 0
    tl_data = 0

    start_time = timer()
    end_time = None
    # end minus start gives time in sec

    if g.remote_joystick:
        print("Note: current service setup may cause networking to fail if it doesnt wait for it. Look into more.")
        print("Setting up sockets...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "127.0.0.1"
        port = 8087

        sock.bind((host, port))
        sock.listen(1)
        client, addr = sock.accept() 
        sendtimer_start = timer()
        sendtimer_end = timer()

        while True: 
            start = timer()
            size = client.recv(1400)
            newPickle = client.recv(1400)
            sendtimer_end = timer()
            try:
                if(sendtimer_end - sendtimer_start) > g.sleep_time:
                    # b = pickle.loads(newPickle)
                    b = newPickle
                    print(b)
                    sp.write(b)
                    sendtimer_start = sendtimer_end
            except EOFError:
                print("eof error occurred in pickle.load")
            except Exception as e: 
                print("Something else went wrong in pickle.load")
                print(e)

            # confirmed looked like serialized data came through and it is
            # a byte array.
            # print(type(b))
            end = timer()
            print(end-start)

    # If its not a remote joystick, do the parsing here
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
                elif event.code == 4:
                    # rjoy y
                    rjoy_y_data = event.value
                elif event.code == 3:
                    #rjoy x
                    rjoy_x_data = event.value
                elif event.code == 1:
                    #ljoy y
                    ljoy_y_data = event.value
                elif event.code == 0:
                    #ljoy x
                    ljoy_x_data = event.value
                elif event.code == 5:
                    # RT
                    tr_data = event.value
                elif event.code == 2:
                    # LT
                    tl_data = event.value
                
        
        #check if its time to send data
        end_time = timer()
        if (end_time - start_time) > g.sleep_time:
            # print(bin(btn_data))
            start_time = end_time
            # send the data

            # store joy message
            proto_msg = uart_messages_pb2.Joystick_Input()

            # create byte array for the data to send
            b = bytearray()

            # store the sync byte
            # length is the number of bytes it will use when it converts count
            b.extend(g.sync_byte.to_bytes(length=1, byteorder='little', signed=False))

            #prepare the proto message to send
            # print("button data", btn_data)
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
            b.extend(msg_len.to_bytes(length=1, byteorder='little', signed = False))
            
            # add the proto data
            b.extend(proto_msg_serialized)
            # print(b)
            sp.write(b)

            # check of reply
            # look for sync byte
            # sp.in_waiting returns the number of bytes in recieve buffer if you want to use that
            if sp.in_waiting > 0:
                first_byte = sp.read(1)[0] #index 0 of byte array to get the value
                if first_byte == 70: # hex 0x46
                    num_bytes = sp.read(1)[0]
                    # print("here2")
                    if num_bytes > 0:
                        # print("here3")
                        data = sp.read(num_bytes)
                        # print(data)
                        # check if received all of the data
                        if len(data) == num_bytes:
                            # print("here4")
                            reply = uart_messages_pb2.GUI_Data()
                            reply.ParseFromString(data)
                            # print(reply.cpu_temp)

                

