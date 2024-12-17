import redis
import cv2
import csv
import json
import numpy as np
import sys
import time
from datetime import datetime
from constants import *

# Socket
SERVER_ADDR = ('localhost', 65432)
REDIS_CLIENT = redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)
CHANNEL = "positions"

r = redis.Redis(host='localhost', port=6379, db=0)
stream_name = 'stream_positions'

STORE_POSITION = False
if len(sys.argv) > 1:
    if sys.argv[1] == "--store-trial":
        STORE_POSITION = True
        try:
            name = EXPERIMENT_NAME
        except NameError:
            name = "test"

        FILE_NAME = f"{name}_{datetime.now().strftime("%H%M%S")}.csv"

print("store time-series: ", STORE_POSITION)

# Calibration and conversion constants
# MIN_X_PIX = 285
# MAX_X_PIX = 1230
# MIN_Y_PIX = 200
# MAX_Y_PIX = 1080
MIN_X_PIX = 360
MAX_X_PIX = 1320
MIN_Y_PIX = 80
MAX_Y_PIX = 1030

SIZE_IN_CM = 30.1625
SIZE_IN_PIX_X = MAX_X_PIX - MIN_X_PIX
SIZE_IN_PIX_Y = MAX_Y_PIX - MIN_Y_PIX
PIX_TO_CM_X = SIZE_IN_CM / SIZE_IN_PIX_X
PIX_TO_CM_Y = SIZE_IN_CM / SIZE_IN_PIX_X
WORLD_ORIGIN_TRANSLATION_X = SIZE_IN_PIX_X / 2
WORLD_ORIGIN_TRANSLATION_Y = SIZE_IN_PIX_Y / 2

# SIZE_IN_PIX_X = MAX_X_PIX - MIN_X_PIX
# SIZE_IN_CM = 30.1625
# PIX_TO_CM = SIZE_IN_CM / SIZE_IN_PIX_X
# WORLD_ORIGIN_TRANSLATION = SIZE_IN_PIX_X / 2

# Color thresholds for black and yellow in RGB
lower_black = np.array([0, 0, 0])
upper_black = np.array([50, 50, 50])
# upper_black = np.array([20, 20, 20])

# red 
lower_red1 = np.array([0, 50, 50])   # Lower bound of red
upper_red1 = np.array([10, 255, 255]) # Upper bound of red (low range)

lower_red2 = np.array([170, 50, 50])  # Lower bound of high red range
upper_red2 = np.array([180, 255, 255]) # Upper bound of high red range

# yellow
# lower_yellow = np.array([0, 100, 100])
# upper_yellow = np.array([100, 255, 255])

# Minimum and maximum contour area thresholds
min_contour_area = 10
max_contour_area = 2000

# min_contour_area = 20
# max_contour_area = 500

def publish_position(writer):
    global start_time

    background_frame = None
    cap = cv2.VideoCapture(0)

    while True:
        _, frame = cap.read()

        if frame is None:
            break

        # Crop the frame
        frame = frame[MIN_Y_PIX:MAX_Y_PIX, MIN_X_PIX:MAX_X_PIX]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (35, 35), 0)

        if background_frame is None:
            background_frame = gray

        frame_delta = cv2.absdiff(background_frame, gray)

        # Threshold the difference image
        _, thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)
        thresh = cv2.dilate(thresh, None, iterations=6)

        black_pos  = [OUT_OF_RANGE, OUT_OF_RANGE]
        yellow_pos = [OUT_OF_RANGE, OUT_OF_RANGE]

        # Detect black objects
        mask_black = cv2.inRange(frame, lower_black, upper_black)
        black_contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Detect red objects
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        yellow_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in yellow_contours:
            if cv2.contourArea(contour) > 10:  # Example contour area threshold
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Draw a red rectangle
                h_pos = ((x + w / 2) - WORLD_ORIGIN_TRANSLATION_X) * PIX_TO_CM_X
                v_pos = (-1 * ((y + h / 2) - WORLD_ORIGIN_TRANSLATION_Y)) * PIX_TO_CM_Y
                yellow_pos = [round(h_pos, 2), round(v_pos, 2)]

        # mask_yellow = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        # mask_yellow = cv2.inRange(frame, lower_yellow, upper_yellow)
        # yellow_contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process black contours
        for cnt in black_contours:
            area = cv2.contourArea(cnt)
            if area < min_contour_area or area > max_contour_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            
            # Calculate aspect ratio
            # aspect_ratio = w / float(h)
            
            # if 0.7 <= aspect_ratio <= 1.3:  # Adjust this range as needed
            # Set a threshold for aspect ratio to detect circular shapes
            # if 0.3 <= aspect_ratio <= 3.5:
            h_pos = ((x + w / 2) - WORLD_ORIGIN_TRANSLATION_X) * PIX_TO_CM_X
            v_pos = (-1 * ((y + h / 2) - WORLD_ORIGIN_TRANSLATION_Y)) * PIX_TO_CM_Y
            black_pos = [round(h_pos, 2), round(v_pos, 2)]

            cv2.rectangle(frame, (x, y), (x + w, y + h), (10, 10, 10), 2)
            # cv2.putText(frame, "black object detected", (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 10, 10), 2)

        # Process yellow contours
        # for cnt in yellow_contours:
        #     area = cv2.contourArea(cnt)
        #     if area < min_contour_area or area > max_contour_area:
        #         continue

        #     x, y, w, h = cv2.boundingRect(cnt)
        #     h_pos = ((x + w / 2) - WORLD_ORIGIN_TRANSLATION_X) * PIX_TO_CM_X
        #     v_pos = (-1 * ((y + h / 2) - WORLD_ORIGIN_TRANSLATION_Y)) * PIX_TO_CM_Y
        #     yellow_pos = [round(h_pos, 2), round(v_pos, 2)]

        #     cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 255, 255), 2)
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            # cv2.putText(frame, "Yellow object detected", (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        payload = {
            "timestamp": round(time.time() - start_time, 3),
            "yellow": json.dumps(yellow_pos),
            "black": json.dumps(black_pos)
        }

        if payload:
            r.xadd("stream_positions", payload)
            if STORE_POSITION:
                print([payload["timestamp"], black_pos[0], black_pos[1], yellow_pos[0], yellow_pos[1]])
                writer.writerow([payload["timestamp"], black_pos[0], black_pos[1], yellow_pos[0], yellow_pos[1]])

        # print and display latest position and frame for diagnostics
        # sys.stdout.write(f"\r blk (in): ({to_in(black_pos[0])}, {to_in(black_pos[1])})")
        # sys.stdout.write(f"\r ylw (in): ({to_in(yellow_pos[0])}, {to_in(yellow_pos[1])})")
        # sys.stdout.write(f"\r Yellow: ({yellow_pos[0]}, {yellow_pos[1]}), cm: ({to_in(yellow_pos[0])}, {to_in(yellow_pos[1])})")
        # sys.stdout.write(f"\r Black: ({black_pos[0]}, {black_pos[1]}) cm: ({to_in(black_pos[0])}, {to_in(black_pos[1])}), Yellow: ({yellow_pos[0]}, {yellow_pos[1]}), cm: ({to_in(yellow_pos[0])}, {to_in(yellow_pos[1])})")
        # sys.stdout.write(f"\r Black: ({black_pos[0]}, {black_pos[1]}) cm: ({to_in(black_pos[0])}, {to_in(black_pos[1])}), Yellow: ({yellow_pos[0]}, {yellow_pos[1]}), cm: ({to_in(yellow_pos[0])}, {to_in(yellow_pos[1])})")
        sys.stdout.flush()
        cv2.imshow('Frame', frame)

        k = cv2.waitKey(5)
        if k == 27:
            break

    cv2.destroyAllWindows()
    cap.release()
    

def to_in(cm):
    return round(cm * 0.393701, 2)

if __name__ == "__main__":
    start_time = time.time()
    # with open(f"final_data/time_series/{FILE_NAME}", mode='w', newline='') as file:
    with open(f"delete_me ", mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'black_x', 'black_y', 'yellow_x', 'yellow_y'])
        publish_position(writer)