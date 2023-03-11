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
