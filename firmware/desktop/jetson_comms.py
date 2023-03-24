#!/usr/bin/python3

import time
import serial
import uart_messages_pb2
import evdev
import sys
from evdev import categorize, ecodes
from timeit import default_timer as timer
import pickle, socket

############################
####      GLOBALS     ######
############################

class g:
    update_rate_serial = 20 # in hz
    sleep_time_serial = 1 / update_rate_serial
    update_rate_readsocket = 30
    sleep_time_readsocket = 1 / update_rate_readsocket
    joy_name = "Logitech Gamepad F710"
    test_input = False
    sync_byte = 100 #0x64
    remote_joystick = False #if true, joystick is connected to base station laptop

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
    first_arg = ""
    second_arg = ""
    if len(sys.argv) == 3:
        first_arg = sys.argv[1].lower()
        second_arg = sys.argv[2].lower()
    elif len(sys.argv) == 2:
        first_arg = sys.argv[1].lower()

    # first arg is local or remote. if remote theres no testing mode
    # if local, then 2nd arg can be testing moding

    if first_arg == "remote":
        g.remote_joystick = True
        g.test_input = False
        print("Joystick is connected to base station, starting up networking to receive data.")
    elif first_arg == "local":
        g.remote_joystick = False
        if second_arg == "test":
            g.test_input = True
            print("In testing mode, will just print controller inputs (no data will be sent)")
        else:
            g.test_input = False
            print("In normal operation mode with joystick connected to the Jetson")

    time.sleep(1.5)

############################
####     MAIN CODE    ######
############################

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

    # check for testing mode
    if g.test_input and not g.remote_joystick:
        joystick.test_inputs()
        # will not return from this loop

    # joystick data
    btn_data = 0
    ljoy_x_data = 0
    ljoy_y_data = 0
    rjoy_x_data = 0
    rjoy_y_data = 0
    tr_data = 0
    tl_data = 0

    ################################
    #### REMOTE JOYSTICK CODE ######
    ################################
    if g.remote_joystick:
        print("Setting up sockets...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "127.0.0.1"
        port = 8087
        sock.bind((host, port))
        sock.listen(1)
        print("Runing socket accept()")
        client, addr = sock.accept() 
        print("Socket accepted.")

        readtimer_start = timer()
        readtimer_end = timer()
        sendtimer_start = timer()
        sendtimer_end = timer()

        sendData = bytearray()

        while True: 
            #loop timer
            start = timer()

            # check if its time to recieve data from the socket
            readtimer_end = timer()
            if(readtimer_end - readtimer_start) > g.sleep_time_readsocket:
                # update timer
                readtimer_start = readtimer_end
                
                # receive the data
                size = client.recv(8)
                expected_length = int.from_bytes(size, byteorder="little")
                sendData = client.recv(expected_length)

                try:
                    if(sendData[0] == 100):
                        # print(sendData)
                        pass
                    else:
                        print("BAD DATA!!")
                        print(sendData)
                        print("Resetting socket..")
                        client.close()
                        client, addr = sock.accept()
                        
                except Exception as e: 
                    print("Exception in receiving data over WiFi")
                    print(e)

            # check if its time to send data to teensy
            if (sendtimer_end - sendtimer_start) > g.sleep_time_serial:
                # update timer
                sendtimer_start = sendtimer_end

                # send it
                print(sendData)
                try:
                    sp.write(sendData)
                except Exception as e:
                    print("Exception occurred in sp.write()")
                    print(e)
                

            # loop timer
            end = timer()
            print("looptimer: ", (end-start))


    ################################
    #### LOCAL JOYSTICK CODE #######
    ################################

    # timers
    # end minus start gives time in sec
    start_time = timer()
    end_time = None

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
        if (end_time - start_time) > g.sleep_time_serial:
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

                

