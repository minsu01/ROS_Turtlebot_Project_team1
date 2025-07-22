import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import termios
import tty
import os
import select
import sys
import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
# -*- coding: utf-8 -*-

def gesture_to_twist(gesture, speed=0.3, turn=0.3):
    """gesture 문자열을 받아 Twist 메시지로 바꿔 줍니다."""
    msg = Twist()

    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    if gesture == "forward":          # paper = forward   보 = 전진
        msg.linear.x = speed
    elif gesture == "stop":         # rock = stop       주먹 → 정지
        msg.linear.x = 0.0
    elif gesture == "right":          # one = right       하나 = 우회전
        msg.angular.z = turn
    elif gesture == "left":       # sissor = left     둘 = 좌회전
       msg.angular.z = -turn
    elif gesture == "reverse":        # three = reverse   셋 = 후진
        msg.linear.x = -speed
    else:                           # 그 외 제스처는 0
        msg.linear.x = 0.0
    return msg

def classify_hand(hand_landmarks):
    results = "I dont know"
    thumb_tip = hand_landmarks.landmark[4]
    index_tip = hand_landmarks.landmark[8]
    middle_tip = hand_landmarks.landmark[12]
    ring_tip = hand_landmarks.landmark[16]
    pinky_tip = hand_landmarks.landmark[20]

    landmarks = hand_landmarks.landmark
    def is_finger_straight(finger_tip_idx, finger_dip_idx):

        return landmarks[finger_tip_idx].y < landmarks[finger_dip_idx].y

    # 손가락 펼침 여부
    index_straight = is_finger_straight(8, 6)
    middle_straight = is_finger_straight(12, 10)
    ring_straight = is_finger_straight(16, 14)
    pinky_straight = is_finger_straight(20, 18)
    thumb_open = landmarks[4].x < landmarks[3].x  # 엄지 위치 판단(개개인 손 방향에 따라 조정 필요)


    if index_straight and not middle_straight and not ring_straight and not pinky_straight:
       results = "right"
    elif index_straight and middle_straight and not ring_straight and not pinky_straight:
        results =  "left"
    elif index_straight and middle_straight and ring_straight and not pinky_straight:
        results =  "reverse"
    elif not index_straight and not middle_straight and not ring_straight and not pinky_straight:
        results =  "stop"
    elif index_straight and middle_straight and ring_straight and pinky_straight:
        results =  "forward"

    return results

rclpy.init()
ros_node = rclpy.create_node('gesture_cmd_vel')
cmd_pub = ros_node.create_publisher(Twist, '/cmd_vel', 10)

cap = cv2.VideoCapture(0)
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("카메라를 찾을 수 없습니다.")
      # 동영상을 불러올 경우는 'continue' 대신 'break'를 사용합니다.
      continue

    # 필요에 따라 성능 향상을 위해 이미지 작성을 불가능함으로 기본 설정합니다.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image)

    # 이미지에 손 주석을 그립니다.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    gesture = "hands detect"

    if results.multi_hand_landmarks:
      for hand_landmarks in results.multi_hand_landmarks:
        mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())

        gesture = classify_hand(hand_landmarks)
    twist_msg = gesture_to_twist(gesture.lower())  # "Rock" → "rock" 맞춰주기
    cmd_pub.publish(twist_msg)

    cv2.putText(image, gesture, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 3)
    cv2.imshow('MediaPipe Hands', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
