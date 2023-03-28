import socket, pickle

class MyClass:
    def __init__(self, name, age):
        self.name = name
        self.age = age

if __name__=='__main__':
    data = MyClass("peter", 8)
    sendData = pickle.dumps(data)
    
    size = int(len(sendData)).to_bytes(8, byteorder='little', signed=False)
    print(size.hex())

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = "127.0.0.1"

    port = 8088

    sock.connect((host, port))
    
    sock.sendall(size)
    sock.sendall(sendData)
    sock.sendall(size)
    sock.sendall(sendData)


    sock.close()
