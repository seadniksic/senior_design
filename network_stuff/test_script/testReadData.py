import socket

class MyClass:
    def __init__(self, name, age):
        self.name = name
        self.age = age

if __name__=='__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = socket.gethostname()

    port = 8087

    sock.bind((host, port))

    sock.listen(1)

    client, addr = sock.accept()

    msg = MyClass(client.recv(1400).decode('utf-8'))

    sock.close()

    print(msg.name)
    print(msg.age)

    sock.close()
    client.close()