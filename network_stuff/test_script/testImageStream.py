import cv2
import numpy as np
import socket
import time
import select

cap = cv2.VideoCapture(0)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8084
sock.connect((host, port))
sockets_list = [sock]
write_sockets = []

fps_limit = 30
delay = 1 / fps_limit

while True:
    startTime = time.time()
    read_sockets, write_sockets, error_sockets = select.select([], sockets_list, [], 5)
    if len(write_sockets) > 0:
        ret, frame = cap.read()

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        ___, sendData = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 25])

        sendData = sendData.tobytes()
        size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
        sock.sendall(size)
        sock.sendall(sendData)
        print(len(sendData))

        

    elapsed_time = time.time() - startTime
    if elapsed_time < delay:
        time.sleep(delay - elapsed_time)


cap.release()
cv2.destroyAllWindows()
