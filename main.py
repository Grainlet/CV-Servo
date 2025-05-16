
--------------------------------------------------------
Hindusthan International Expo Project 1 2024-2025
Last Commit Note: Change COM Channel & D Pin
--------------------------------------------------------

--------------------------------------------------------
Dependancies:
#opencv-python 
#mediapipe
#pyfirmata
-------------------------------------------------------
import cv2
import mediapipe as mp
from pyfirmata import Arduino
from collections import deque
import time

board = Arduino('COM3')
servo = board.get_pin('d:9:s')
time.sleep(2)

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

buffer = deque(maxlen=20)
prev_direction = None
servo_pos = 90
servo.write(servo_pos)

def get_direction(points):
    total = 0
    for i in range(len(points) - 2):
        x1, y1 = points[i]
        x2, y2 = points[i+1]
        x3, y3 = points[i+2]
        area = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
        total += area
    return 'ccw' if total > 0 else 'cw'

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            h, w, _ = frame.shape
            tip = hand_landmarks.landmark[8]
            tip_pos = (int(tip.x * w), int(tip.y * h))

            buffer.append(tip_pos)

            for i in range(1, len(buffer)):
                cv2.line(frame, buffer[i-1], buffer[i], (0, 255, 0), 2)

            if len(buffer) >= 10:
                direction = get_direction(list(buffer))
                if direction != prev_direction:
                    print(f"Rotation: {direction}")
                    if direction == 'cw':
                        servo_pos = max(0, servo_pos - 5)
                    else:
                        servo_pos = min(180, servo_pos + 5)
                    servo.write(servo_pos)
                    prev_direction = direction

    cv2.imshow("Index Rotation Control", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
board.exit()
