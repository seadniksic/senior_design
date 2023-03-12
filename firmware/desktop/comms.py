import time
import serial
import uart_messages_pb2

if __name__ == "__main__":
    
    # arduino defualt is 8 data bits, no parity, and one stop bit
    # create sp= serial port object
    sp = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    time.sleep(1)

    sync_byte = 100 # 0x64

    while True:

        # store joy message
        proto_msg = uart_messages_pb2.Joystick_Input()

        # create byte array for the data to send
        b = bytearray()

        # store the sync byte
        # length is the number of bytes it will use when it converts count
        b.extend(sync_byte.to_bytes(length=1, byteorder='little', signed=False))

        #prepare the proto message to send
        proto_msg.button = uart_messages_pb2.Joystick_Input.BTN_TR #| uart_messages_pb2.Joystick_Input.BTN_TR
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

    # code below will not run
    exit()

    count = 0
    
    while True:

        proto_msg = uart_messages_pb2.Command()
        proto_msg.value = count
        
        # make a byte array
        b = bytearray()

        # serialize the message
        proto_msg_serialized = proto_msg.SerializeToString()

        #throw it in the byte array
        b.extend(proto_msg_serialized)

        #wrie the data
        sp.write(b)

        # length is the number of bytes it will use when it converts count
        # b.extend(proto_msg.value.to_bytes(length=4, byteorder='little', signed=False))


        count+=5
        time.sleep(1)
