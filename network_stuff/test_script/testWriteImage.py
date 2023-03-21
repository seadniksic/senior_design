import socket, pickle
import cv2

if __name__=='__main__':
    data = cv2.imread('testImage_3.jpg')
    data = data.tobytes()
    size = int(len(data)).to_bytes(8, byteorder='little', signed=False)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = "127.0.0.1"

    port = 8088

    sock.connect((host, port))
    
    sock.sendall(size)
    sock.sendall(data)

    sock.close()