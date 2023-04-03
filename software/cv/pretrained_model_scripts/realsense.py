import sys
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
import socket
from cv_bridge import CvBridge
import math
import mediapipe as mp
import tensorflow as tf

model_save_path = "models/movenet_multipose_lightning_1"
model = tf.saved_model.load(model_save_path)
tf.debugging.set_log_device_placement(True)
movenet = model.signatures['serving_default']

bridge = CvBridge()
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "127.0.0.1"
port = 8084
sock.connect((host, port))




window_title = "Stream"
start = time.time()

input_size = 256
# capture_width = 1280
# capture_height = 720
confidence_threshold = .3

EDGES = {
    (0, 1): 'm',
    (0, 2): 'c',
    (1, 3): 'm',
    (2, 4): 'c',
    (0, 5): 'm',
    (0, 6): 'c',
    (5, 7): 'm',
    (7, 9): 'm',
    (6, 8): 'c',
    (8, 10): 'c',
    (5, 6): 'y',
    (5, 11): 'm',
    (6, 12): 'c',
    (11, 12): 'y',
    (11, 13): 'm',
    (13, 15): 'm',
    (12, 14): 'c',
    (14, 16): 'c'
}

colors = {0: (0,0,255),
          1: (0,255,0),
          2: (255,0,0),
          3: (0,255,255),
          4: (255,0,255),
          5: (255,255,255)}

#window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE )


def detect_pose(frame):
    #print("hello")
           
    #with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    try:
        # Window
        # now = time.time()

        # image_tensor = tf.cast(tf.image.resize_with_pad(np.expand_dims(frame, axis=0), 192, 192), dtype=tf.int32)

        # # # Output: [1, 6, 56] tensor that contains keypoints/bbox/scores.
        # keypoints_with_scores = movenet(image_tensor)["output_0"]

        # # # print(keypoints_with_scores)
        # # # print(type(keypoints_with_scores))

        # draw_pose(keypoints_with_scores.numpy(), frame)
            
        # frame.flags.writeable = False
        # results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        #print(type(frame))

        #print(results.pose_landmarks)
        #print(frame.shape)
        temp = np.reshape(frame, (1, *frame.shape))
        #print(temp.shape)
        prepared_frame = tf.convert_to_tensor(temp, dtype=tf.uint8)
        #print(prepared_frame.shape)
        resized_image, image_shape = keep_aspect_ratio_resizer(prepared_frame, input_size)
        image_tensor = tf.cast(resized_image, dtype=tf.int32)

        # # Output: [1, 6, 56] tensor that contains keypoints/bbox/scores.
        keypoints_with_scores = movenet(image_tensor)["output_0"]

        #print(keypoints_with_scores.shape)

        # # print(keypoints_with_scores)
        # # print(type(keypoints_with_scores))

        draw_pose(keypoints_with_scores.numpy(), frame)

        # if results.pose_landmarks:
        #     mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
            #cv2.putText(frame, f"{fps:.2}", (int(frame.shape[1] * 5 / 6), int(frame.shape[0] * 5 / 6)), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        #cv2.imshow(window_title, frame)
        #print(type(frame))
        return frame
    except Exception as e:
        print(e)
        return

def draw_pose(keypoints, frame):
  keypoints = keypoints[:, :, :51].reshape((6, 17,3))

  #print(keypoints.shape)
  
  for index,person in enumerate(keypoints):

    y,x,c = frame.shape
    shaped = np.squeeze(np.multiply(person,[y,x,1]))

    running_sum = 0
    for kp in shaped:
      running_sum += kp[2]
    avg_score = running_sum / shaped.shape[0]

    if avg_score > confidence_threshold:
      for kp in shaped:
        ky,kx,kp_conf = kp
        cv2.circle(frame, (int(kx), int(ky)), 4, color=colors[index], thickness=-1)
      for edge, color in EDGES.items():
        p1, p2 = edge
        y1, x1, c1 = shaped[p1]
        y2, x2, c2 = shaped[p2] 
            
        cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,0,255), 4)

def keep_aspect_ratio_resizer(image, target_size):
  """Resizes the image.

  The function resizes the image such that its longer side matches the required
  target_size while keeping the image aspect ratio. Note that the resizes image
  is padded such that both height and width are a multiple of 32, which is
  required by the model.
  """
  #print(image.shape)
  _, height, width, _ = image.shape
  if height > width:
    scale = float(target_size / height)
    target_height = target_size
    scaled_width = math.ceil(width * scale)
    image = tf.image.resize(image, [target_height, scaled_width])
    target_width = int(math.ceil(scaled_width / 32) * 32)
  else:
    scale = float(target_size / width)
    target_width = target_size
    scaled_height = math.ceil(height * scale)
    image = tf.image.resize(image, [scaled_height, target_width])
    target_height = int(math.ceil(scaled_height / 32) * 32)
  image = tf.image.pad_to_bounding_box(image, 0, 0, target_height, target_width)
  return (image,  (target_height, target_width))

def image_callback(msg):
    global start
    #print("gets message")
    pose = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #print("gets message")
    
    #now = time.time()
    frame_with_pose = detect_pose(pose)

    # cv2.imshow(window_title, pose)
    # cv2.waitKey(1)

    # print(frame_with_pose.shape)
    # print(type(frame_with_pose))
    # print(frame_with_pose.dtype)
    
    __, sendData = cv2.imencode('.jpg', pose, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    sendData = sendData.tobytes()
    size = int(len(sendData)).to_bytes(8, byteorder="little", signed=False)
    #print("SENT", len(sendData))
    sock.sendall(size)
    sock.sendall(sendData)
    print(f"fps: {1/(time.time()-start)}")
    start = time.time()



rospy.init_node("image_subscriber")
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.spin()
sock.close()
