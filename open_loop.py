from actuator import Actuator
from constants import *
import asyncio
import ast
import csv
import redis
import time

CSV_FILE = "two_fish_open_loop_position_time_series_real_1.csv"
# YELLOW_POLICY  = {"color": "yellow", "orbit": [112, 128, 144, 128]}
# BLACK_POLICY = {"color": "black", "orbit": [31, 32, 33, 34]}

# CSV_FILE = "two_fish_open_loop_position_time_series_real_1.csv"
# YELLOW_POLICY  = {"color": "yellow", "orbit": [65, 66, 67, 68, 83, 98, 113, 114, 115, 116, 131, 146, 145, 144, 159, 158, 157, 156, 141, 126, 111, 110, 95, 80]}
# BLACK_POLICY = {"color": "black", "orbit": [31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 55, 70, 85, 86, 87, 88, 103, 118, 133, 148, 163, 178, 193, 192, 191, 190, 189, 188, 187, 186, 185, 184, 169, 154, 153, 152, 151, 136, 121, 106, 91, 76, 61, 46]}


# BLACK_POLICY = {"color": "black", "orbit": [80, 81, 82, 97, 112, 113, 114, 129, 144, 143, 142, 141, 140, 125, 110, 95]}
# YELLOW_POLICY  = {"color": "yellow", "orbit": [20, 21, 22, 23, 23, 24, 25, 26, 41, 56, 71, 89, 101, 116, 115, 114, 113, 112, 111, 110, 95, 80, 79, 78, 63, 48, 49, 50, 35]}

# BLACK_POLICY = {"color": "black", "orbit": [144, 145, 146, 161, 176, 175, 174, 159, 144]} # good: inner square (dry acrylic) 
# YELLOW_POLICY  = {"color": "yellow", "orbit": [112, 113, 114, 115, 116, 117, 118, 133, 148, 163, 178, 193, 208, 207, 206, 205, 204, 203, 202, 187, 172, 157, 144, 127]}

BLACK_POLICY = {"color": "black", "orbit": [114, 115, 116, 131, 146, 145, 144, 129]} 
YELLOW_POLICY  = {"color": "yellow", "orbit": [82, 83, 84, 85, 86, 87, 88, 103, 118, 133, 148, 163, 178, 177, 176, 175, 174, 189, 204, 203, 202, 201, 200, 185, 170, 155, 156, 157, 142, 127, 112, 97]}

csv_writer, start_time = None, None
black_ref_idx, yellow_ref_idx = BLACK_POLICY["orbit"][0], YELLOW_POLICY["orbit"][0]

GRID_WIDTH = 15

def idx_to_grid(idx):
    row = idx // GRID_WIDTH
    col = idx % GRID_WIDTH
    return row, col

def read_positions(ipc_client, idx):
    global yellow_ref_idx, black_ref_idx
    messages = ipc_client.xrevrange(POSITIONS_STREAM, count=1)
    if messages:
        _, message = messages[0]
        timestamp = time.time() - start_time
        yellow_position = ast.literal_eval(message[b'yellow'].decode('utf-8'))
        black_position = ast.literal_eval(message[b'black'].decode('utf-8'))
        print(timestamp, yellow_ref_idx, yellow_position[0], yellow_position[1], black_ref_idx, black_position[0], black_position[1])
        csv_writer.writerow([timestamp, yellow_ref_idx, yellow_position[0], yellow_position[1], black_ref_idx, black_position[0], black_position[1]])

async def actuate_sequence(actuator, policy, ipc_client, duration=DEFAULT_ACTUATION_DURATION):
    global black_ref_idx, yellow_ref_idx
    while(True):
        orbit = policy["orbit"]
        for idx in orbit:
            await actuator.actuate_single(*idx_to_grid(idx), duration)

            if policy["color"] == "black":
                black_ref_idx = idx
            elif policy["color"] == "yellow":
                yellow_ref_idx = idx

            read_positions(ipc_client, idx)

async def main(ipc_client):
    global csv_writer, start_time, black_ref_idx, yellow_ref_idx
    start_time = time.time()

    csv_file = open(CSV_FILE, mode="w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["timestamp","yellow_ref_idx", "yellow_position_x", "yellow_position_y", "black_ref_idx", "black_position_x", "black_position_y"])

    with Actuator("/dev/cu.usbmodem21401") as actuator:
        print(ipc_client)    
        print("start concurrent actuation...")
        task1 = asyncio.create_task(actuate_sequence(actuator, BLACK_POLICY, ipc_client, 0.2))
        task2 = asyncio.create_task(actuate_sequence(actuator, YELLOW_POLICY, ipc_client, 0.2))
        # await asyncio.gather(task1)
        # await asyncio.gather(task2)
        await asyncio.gather(task1, task2)

    csv_file.close()

if __name__ == "__main__":
    with redis.Redis(host='localhost', port=6379, db=0) as ipc_client:
        asyncio.run(main(ipc_client))

# from actuator import Actuator
# import time

# GRID_WIDTH = 15

# def idx_to_grid(idx):
#     row = idx // GRID_WIDTH
#     col = idx % GRID_WIDTH
#     return row, col

# if __name__ == "__main__":
#     print("correct script")
#     with Actuator("/dev/cu.usbmodem21401") as actuator:
#         while True:
#             for idx in YELLOW_ORBIT:
#                 actuator.actuate_single_sync(*idx_to_grid(idx))