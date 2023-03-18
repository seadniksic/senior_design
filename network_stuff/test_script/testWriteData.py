import socket
import sys

class MyClass:
    def __init__(self, name, age):
        self.name = name
        self.age = age

if __name__=='__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = socket.gethostname()

    port = 8088

    sock.bind((host, port))

    data = MyClass("peter", 8)
    size = sys.getsizeof(data)

    sock.sendall(size.encode())
    sock.sendall(data.encode())

    sock.close()
