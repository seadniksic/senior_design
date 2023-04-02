#!/usr/bin/python3

import time
import serial
import uart_messages_pb2
import evdev
import sys
from evdev import categorize, ecodes
from timeit import default_timer as timer
import pickle, socket
import matplotlib.pyplot as plt
import numpy as np

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
#####  SERIAL WRAPPER  #####
############################

class Serial_Wrapper:
    def __init__(self):
        # create sp= serial port object
        # arduino defualt is 8 data bits, no parity, and one stop bit
        self.sp = serial.Serial(
            port="/dev/ttyTHS0",
            baudrate=500000,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )

    def read(self, numbytes=1) -> bytearray:
        b = bytearray
        try:
            return self.sp.read(numbytes)
        except Exception as e:
            print("Exception occured in read")
            print(e)
        return b

    def write(self, b : bytearray):
        try:
            self.sp.write(b)
        except Exception as e:
            print("Exception occured in write")
            print(e)

    def in_waiting(self) -> int:
        try:
            return self.sp.in_waiting
        except Exception as e:
            print("Exception occured in in_waiting")
            print(e)
            return 0

    


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

    print("WAS ttyTHS0 configured with chmod to 0666??")
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
    sp = Serial_Wrapper()

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

        sendSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sendSock.connect((host, 8086))

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
            sendtimer_end = timer()
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

            # Read serial port as fast as possible:
            # check if data available
            if sp.in_waiting() < 1:
                continue

            # check for sync byte
            first_byte = sp.read(1)[0]
            if first_byte != 70: #hex 0x46
                continue

            # check for msg type
            second_byte = sp.read(1)[0]
            if second_byte == 102: #hex 0x66
                # check if msg length is nonzero
                num_bytes = sp.read(1)[0]
                if num_bytes < 1:
                    continue
                
                # read data
                data = sp.read(num_bytes)
                sendDataLength = int(len(data)).to_bytes(8, byteorder='little', signed=False)
                sendSock.sendall(sendDataLength)
                sendSock.sendall(data)


            elif second_byte == 68: # hex 0x44
                # check length
                num_bytes = sp.read(1)[0]
                if num_bytes < 1:
                    continue
                
                # read data
                data = sp.read(num_bytes)
                

                # check if received all of the data
                if len(data) == num_bytes:
                    end_slam_timer = timer()
                    reply = uart_messages_pb2.SLAM_Data()
                    try: 
                        reply.ParseFromString(data)
                        # print(end_slam_timer-start_slam_timer,num_bytes ,reply.lia_x, reply.lia_y, reply.lia_z, reply.eul_y, reply.eul_p,reply.eul_r, reply.pan, reply.tilt)
                    except Exception as e:
                        print("corrupt message, failed to serialize SLAM_DATA")
                        print(e)

                    
                    # print(end_slam_timer-start_slam_timer)
                    # np.append(lin_x,reply.lia_x)
                    # line1, = ax.plot(sample,lin_x, 'r') 
                    # line1.set_ydata(lin_x)
                    # fig.canvas.draw()
                    # fig.canvas.flush_events()
                    start_slam_timer = timer()

            end_loop_timer = timer()
            # print("LOOP_TIME: ", end_loop_timer-start_loop_timer)
                

            # loop timer
            end = timer()
            # print("looptimer: ", (end-start))


    ################################
    #### LOCAL JOYSTICK CODE #######
    ################################

    # timers
    # end minus start gives time in sec
    start_time = timer()
    end_time = None

    # plotting
    # sample = np.linspace(0, 200, 200, dtype = int)
    # lin_x = np.zeros(sample.size, dtype = float)
    # lin_y = np.zeros(sample.size, dtype = float)
    # lin_z = np.zeros(sample.size, dtype = float)
    # eul_y = np.zeros(sample.size, dtype = float)
    # eul_p = np.zeros(sample.size, dtype = float)
    # eul_r = np.zeros(sample.size, dtype = float)

    # plt.ion()
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # line1, = ax.plot(sample, lin_x, 'b-')
    # line2, = ax.plot(sample, lin_y, 'b-')

    start_slam_timer = timer()
    end_slam_timer = timer()

    start_loop_timer = timer()
    end_loop_timer = timer()

    test_latency_start = timer()
    test_latency_end = timer()

    valid = True


    # If its not a remote joystick, do the parsing here
    while True: 
        start_loop_timer = timer()
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

            og_size = 28
            print(f"msg_len = {msg_len}, og_size={og_size}")
            print(f"COMPRESSION: {(msg_len*100 / og_size)}")

            
            # print("proto msg", proto_msg_serialized)
            # print("msg length",msg_len)
            b.extend(msg_len.to_bytes(length=1, byteorder='little', signed = False))
            
            # add the proto data
            b.extend(proto_msg_serialized)
            
            print("SENT DATA:")
            print(b)
            sp.write(b)
            test_latency_start = timer()
            valid = True

            


        if(sp.in_waiting() < 1):
            continue

        print("RECIEVE DATA:")

        first_byte = sp.read(1)[0]

        if first_byte != 70: #hex 0x46
            print("WRONG SYNC BYTE")
            continue

        while(sp.in_waiting() < 1):
            pass

        num_bytes = sp.read(1)[0]
        if num_bytes < 1:
            print("WRONG SIZE")
            continue

        data = sp.read(num_bytes)
        print(data)
        if valid is True:
            test_latency_end = timer()
            print("LATENCY: ", test_latency_end - test_latency_start)
            valid = False
        else:
            print("LATENCY INVALID")

        if len(data) == num_bytes:
            reply = uart_messages_pb2.Joystick_Input()
            reply.ParseFromString(data)
            print(reply)



        
