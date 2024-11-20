import pdb
import asyncio
import numpy as np
import matplotlib.pyplot as plt
import redis
import time
from agent import Agent
from magtile_platform import Platform
from actuator import Actuator
from constants import *
from plotter import Plotter


def perform_control(ipc_client=None):
    platform = Platform(ipc_client)
    plotter = Plotter(platform)
    plotter.update_plot()

    global i 
    i = 0

    while True:
        print(f"\n--- control loop: {i} ----")
        platform.current_control_iteration = i

        platform.reset_interference_parameters()
        platform.update_agent_positions()

        if CONTROL_MODE == "HALTING":
            platform.perform_halting_collision_avoidance()
        elif CONTROL_MODE == "STEERING":
            platform.perform_steering_collision_avoidance()
        else:
            raise "invalid control strategy: ['HALTING', 'STEERING']"

        asyncio.run(platform.advance_agents())
        
        # platform.alert_if_collision()
        plotter.update_plot()
        i += 1

        if OPERATION_MODE == "LIVE":
            time.sleep(0.3)
        # pdb.set_trace()

    plt.ioff()  # Turn off interactive mode when done
    plt.show()

if __name__ == "__main__":
    if OPERATION_MODE == "SIMULATION":
        perform_control()
    
    elif OPERATION_MODE == "LIVE":
        with Actuator("/dev/cu.usbmodem21401") as actuator:
            with redis.Redis(host='localhost', port=6379, db=0) as ipc_client:
                Agent.set_actuator(actuator)
                perform_control(ipc_client)
    
    else:
        raise "invalid operation mode: ['SIMULATION', 'LIVE']"