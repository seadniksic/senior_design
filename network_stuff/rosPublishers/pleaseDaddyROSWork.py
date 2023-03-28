#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import String
import socket, select
import numpy as np
import pickle

class ImagePublisher(Node):
    def __init__(self):