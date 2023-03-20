import socket, sys, pickle

class MyClass:
    def __init__(self, name, age):
        self.name = name
        self.age = age

if __name__=='__main__':
    data = MyClass("peter", 8)
    sendData = pickle.dumps(data)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = socket.gethostname()

    port = 8088

    sock.connect((host, port))

    sock.sendall(bytes(str(len(sendData)), 'utf-8'))
    sock.sendall(sendData)

    sock.close()
