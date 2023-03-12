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

interpreter = tf.lite.Interpreter(model_path='models/movenet.tflite')

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



def detect(interpreter, input_tensor):
  """Runs detection on an input image.

  Args:
    interpreter: tf.lite.Interpreter
    input_tensor: A [1, input_height, input_width, 3] Tensor of type tf.float32.
      input_size is specified when converting the model to TFLite.

  Returns:
    A tensor of shape [1, 6, 56].
  """

  input_details = interpreter.get_input_details()
  output_details = interpreter.get_output_details()

  is_dynamic_shape_model = input_details[0]['shape_signature'][2] == -1
  if is_dynamic_shape_model:
    input_tensor_index = input_details[0]['index']
    input_shape = input_tensor.shape
    interpreter.resize_tensor_input(
        input_tensor_index, input_shape, strict=True)
  interpreter.allocate_tensors()

  interpreter.set_tensor(input_details[0]['index'], input_tensor.numpy())

  interpreter.invoke()

  keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
  return keypoints_with_scores

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

colors = {0: (0,0,255),
          1: (0,255,0),
          2: (255,0,0),
          3: (0,255,255),
          4: (255,0,255),
          5: (255,255,255)}

def drawPose(keypoints, frame):
  keypoints = keypoints[:, :, :51].reshape((6, 17,3))
  
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
  

def show_camera():

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
          while count < 250:
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
                      
                      #shape = frame.shape
                      prepared_frame = tf.convert_to_tensor(np.reshape(frame, (1, *frame.shape)), dtype=tf.float32)
                      resized_image, image_shape = keep_aspect_ratio_resizer(prepared_frame, input_size)
                      image_tensor = tf.cast(resized_image, dtype=tf.uint8)

                      # Output: [1, 6, 56] tensor that contains keypoints/bbox/scores.
                      keypoints_with_scores = detect(
                            interpreter, tf.cast(image_tensor, dtype=tf.uint8))
                          
                      # if not keypoints_with_scores:
                      #     cv2.imshow(window_title, frame)
                      #     continue
                          
                  
                          #mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

                      
                      drawPose(keypoints_with_scores, frame)
                      
                      cv2.putText(frame, f"{fps:.2}", (int(frame.shape[1] * 5 / 6), int(frame.shape[0] * 5 / 6)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                      cv2.imshow(window_title, frame)
                    
              else:
                  break
              
              keyCode = cv2.waitKey(10) & 0xFF
              # Stop the program on the ESC key or 'q'
              if keyCode == 27 or keyCode == ord('q'):
                  break
              
          print(f"Average FPS: {rsum / 150}")
      finally:
          video_capture.release()
          cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == "__main__":

    show_camera()
