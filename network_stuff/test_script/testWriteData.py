import socket, sys, pickle

class MyClass:
    def __init__(self, name, age):
        self.name = name
        self.age = age

if __name__=='__main__':
    data = MyClass("peter", 8)
    size = sys.getsizeof(data)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = socket.gethostname()

    port = 8088

    sock.connect((host, port))

    sendData = pickle.dumps(size)
    sock.sendall(sendData)
    sendData = pickle.dumps(data)
    sock.sendall(sendData))

    sock.close()
