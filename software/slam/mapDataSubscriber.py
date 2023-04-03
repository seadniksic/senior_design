import rospy, socket, pickle, zstd
from rtabmap_ros.msg import MapData
from rosMapToRos2Map import Ros1MapDataBridge

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8085
sock.connect((host, port))

def map_data_callback(msg):
    msgData = Ros1MapDataBridge(msg)
    sendData = pickle.dumps(msgData)

    sendData = zstd.compress(sendData)

    size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    sock.sendall(size)
    sock.sendall(sendData)

rospy.init_node("mapSub")
rospy.Subscriber("/rtabmap/", MapData, map_data_callback)
rospy.spin()
sock.close()