import socket
import sys

class MyClass:
    def __init__(self, name, age):
        self.name = name
        self.age = age

if __name__=='__main__':
    data = MyClass("peter", 8)
    size = sys.getsizeof(data)
    print(size)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = socket.gethostname()

    port = 8088

    sock.connect((host, port))

    sock.sendall(repr(size).encode('utf-8'))
    sock.sendall(repr(data).encode('utf-8'))

    sock.close()
