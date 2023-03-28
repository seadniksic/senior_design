import socket, pickle

class MyClass:
    def __init__(self, name, age):
        self.name = name
        self.age = age

if __name__=='__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = "127.0.0.1"

    port = 8087

    sock.bind((host, port))

    sock.listen(1)

    client, addr = sock.accept()

    size = client.recv(1400)
    msg = client.recv(1400)
    msg = pickle.loads(msg)

    sock.close()

    print(msg.name)
    print(msg.age)

    sock.close()
    client.close()