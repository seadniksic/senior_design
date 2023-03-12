import time
from time import sleep
import serial
import uart_messages_pb2
import evdev
# import atexit
import sys
from evdev import categorize, ecodes
from timeit import default_timer as timer
import struct

class g:
    update_rate = 1 # in hz
    sleep_time = 1/ update_rate
    joy_name = "Logitech Gamepad F710"
    test_input = False
    sync_byte = 100 #0x64

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
            # omitting DPAD
            318 : 1024,
            317 : 2048,
            311 : 4096,
            310 : 8192
        }

        self.DPAD_LUT = {
            "UP" : 64,
            "DOWN" : 128,
            "LEFT" : 256,
            "RIGHT" : 512
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
    if len(sys.argv) > 1: 
        first_arg = sys.argv[1].lower()
        if first_arg == "true":
            g.test_input = True
            print("In testing mode, will just print controller inputs (no data will be sent)")
        
    if not g.test_input:
        print("In normal operation mode.")

    sleep(1.5)


if __name__ == "__main__":

    # get joystick
    joystick = Joystick()

    if joystick.dev is None:
        print("JOYSTICK NOT FOUND!! EXITING...")
        exit()
    joystick.print_dev_capabilities(False)
    

    # handle script args and update g class with values
    handle_args()

    # create sp= serial port object
    # arduino defualt is 8 data bits, no parity, and one stop bit
    sp = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    time.sleep(1)

    if g.test_input:
        joystick.test_inputs()

    # always update the struct
    # but send at 10hz
    # if struct doesnt update, no need to send data!
    # read device inputs

    # H means unsigned short so uint16_t
    # data = struct.pack("H", 0)

    btn_data = 0

    start_time = timer()
    end_time = None
    # end minus start gives time in sec

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
                    pass
                    # if event.value == 1:
                    #     btn_data = btn_data | joystick.DPAD_LUT["RIGHT"] 
                
        
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
            print("button data", btn_data)
            proto_msg.button = btn_data
            # serialize the data
            proto_msg_serialized = proto_msg.SerializeToString()

            # store the length of the message and add it to be sent
            msg_len = len(proto_msg_serialized)
            # fix it sending nothing when data is equal to zero
            if(msg_len == 0):
                msg_len = 2
                b.extend(msg_len.to_bytes(length=1, byteorder='little', signed = False))
                zero = 0
                b.extend(zero.to_bytes(length=2, byteorder = 'little', signed= False))
            else: 
                # print("proto msg", proto_msg_serialized)
                # print(msg_len)
                b.extend(msg_len.to_bytes(length=1, byteorder='little', signed = False))
                
                # add the proto data
                b.extend(proto_msg_serialized)
            print(b)
            sp.write(b)
    



    while True:

        # store joy message
        proto_msg = uart_messages_pb2.Joystick_Input()

        # create byte array for the data to send
        b = bytearray()

        # store the sync byte
        # length is the number of bytes it will use when it converts count
        b.extend(g.sync_byte.to_bytes(length=1, byteorder='little', signed=False))

        #prepare the proto message to send
        proto_msg.button = uart_messages_pb2.Joystick_Input.BTN_TR | uart_messages_pb2.Joystick_Input.DPAD_LEFT
        # serialize the data
        proto_msg_serialized = proto_msg.SerializeToString()

        # store the length of the message and add it to be sent
        msg_len = len(proto_msg_serialized)
        print(proto_msg_serialized)
        print(msg_len)
        b.extend(msg_len.to_bytes(length=1, byteorder='little', signed = False))
        
        # add the proto data
        b.extend(proto_msg_serialized)
        sp.write(b)


        time.sleep(1)

