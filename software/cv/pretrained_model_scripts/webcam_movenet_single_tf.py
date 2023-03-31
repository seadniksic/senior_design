'''
Author:  Sead Niksic
Code inspired by: https://github.com/jetsonhacks/USB-Camera/blob/main/usb-camera-simple.py && https://google.github.io/mediapipe/solutions/pose#python-solution-api
&& tf documentation https://tfhub.dev/google/lite-model/movenet/multipose/lightning/tflite/float16/1
&& poseDrawing from https://github.com/nicknochnack/MultiPoseMovenetLightning/blob/main/MultiPose%20MoveNet%20Tutorial.ipynb


'''

import sys
import cv2
import numpy as np
import time
import tensorflow as tf
import math


window_title = "Stream"
model_save_path = "models/movenet_single"
model = tf.saved_model.load(model_save_path)
#tf.debugging.set_log_device_placement(True)
movenet = model.signatures['serving_default']

input_size = 256
capture_width = 1280
capture_height = 720
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


def detect_pose():

    camera_id = "/dev/video0"

    # webcams -> V4L2, stereo -> gstreamer
    video_capture = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)

    rsum = 0
    count = 0
    if video_capture.isOpened():
      try:
          window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE )
          # Window
          now = time.time()
          while count < 600:
              count += 1
              fps = 1 / (time.time() - now)
              if count != 1:
                  rsum += fps

              now = time.time()
                  
              ret_val, frame = video_capture.read()

              # Check to see if the user closed the window
              # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
              # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
              if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                  if not ret_val:
                      continue
                      
                  else:
                      
                      # #shape = frame.shape
                      # with tf.device('/GPU:0'):
                    #   prepared_frame = tf.convert_to_tensor(np.reshape(frame, (1, *frame.shape)), dtype=tf.uint8)
                    #   resized_image, image_shape = keep_aspect_ratio_resizer(prepared_frame, input_size)
                      image_tensor = tf.cast(tf.image.resize_with_pad(np.expand_dims(frame, axis=0), 192, 192), dtype=tf.int32)

                      # # Output: [1, 6, 56] tensor that contains keypoints/bbox/scores.
                      keypoints_with_scores = movenet(image_tensor)["output_0"]

                      # # print(keypoints_with_scores)
                      # # print(type(keypoints_with_scores))

                      draw_pose(keypoints_with_scores.numpy(), frame)
                      
                      cv2.putText(frame, f"{fps:.2}", (int(frame.shape[1] * 5 / 6), int(frame.shape[0] * 5 / 6)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                      cv2.imshow(window_title, frame)
                    
              else:
                  break
              
              keyCode = cv2.waitKey(10) & 0xFF
              # Stop the program on the ESC key or 'q'
              if keyCode == 27 or keyCode == ord('q'):
                  break
              
          print(f"Average FPS: {rsum / count}")
      finally:
          video_capture.release()
          cv2.destroyAllWindows()
    else:
        print("Unable to open camera")



def draw_pose(keypoints, frame):
  #keypoints = keypoints[:, :, :51].reshape((6, 17,3))
  
  #for index,person in enumerate(keypoints):

    y,x,c = frame.shape
    shaped = np.squeeze(np.multiply(keypoints,[y,x,1]))

    running_sum = 0
    for kp in shaped:
        running_sum += kp[2]
    avg_score = running_sum / shaped.shape[0]

    if avg_score > confidence_threshold:
        for kp in shaped:
            ky,kx,kp_conf = kp
            cv2.circle(frame, (int(kx), int(ky)), 4, color=(255,0,0), thickness=-1)
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




if __name__ == "__main__":

    detect_pose()
