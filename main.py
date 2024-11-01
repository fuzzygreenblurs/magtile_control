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

if __name__ == '__main__':
    if OPERATION_MODE == "SIMULATION":
        platform = Platform()
        plotter = Plotter(platform)
        plotter.update_plot()

        global i 
        i = 0

        while True:
            print(f"\n--- control loop: {i} ----")
            platform.current_control_iteration = i  

            # print("BLACK input trajectory: ", platform.black_agent.input_trajectory[i:i+10])
            
            platform.reset_interference_parameters()
            platform.update_agent_positions()

            if CONTROL_MODE == "HALTING":
                platform.perform_halting_collision_avoidance()
            elif CONTROL_MODE == "STEERING":
                platform.perform_steering_collision_avoidance()
            else:
                raise "invalid control strategy: ['HALTING', 'STEERING']"

            asyncio.run(platform.advance_agents())

            plotter.update_plot()
            time.sleep(0.3)
            i += 1

        # plt.ioff()  # Turn off interactive mode when done
        # plt.show()
        
    else:
        pass
        # with Actuator("/dev/cu.usbmodem21301") as actuator:
        #     with redis.Redis(host='localhost', port=6379, db=0) as ipc_client:
        #         Agent.set_actuator(actuator)
        #         platform = Platform(ipc_client)
        #         # print("initial state: ")
        #         # for a in platform.agents:
        #         #         print(f"{a.color}: {a.position}")

        #         black_ref_trajectory = platform.black_agent.ref_trajectory
        #         black_input_trajectory = platform.black_agent.shortest_path
        #         yellow_ref_trajectory = platform.yellow_agent.ref_trajectory
        #         yellow_input_trajectory = platform.yellow_agent.shortest_path

        #         update_trajectories = run_plot(black_input_trajectory, yellow_input_trajectory, black_ref_trajectory, yellow_ref_trajectory, platform)

        #         i = 0

        #         while True:
        #             platform.current_control_iteration = i           
        #             print(f"\n--- control loop: {i} ----")  

        #             platform.reset_interference_parameters()
        #             platform.update_agent_positions()


        #             print("black position: ", platform.black_agent.position)
        #             print("yellow position: ", platform.yellow_agent.position)

        #             platform.plan_for_interference()

        #             # print("black input trajectory: ", platform.black_agent.input_trajectory[i:i+4])
        #             # print("yellow input trajectory: ", platform.yellow_agent.input_trajectory[i:i+4])

        #             asyncio.run(platform.advance_agents())

        #             black_input_trajectory = platform.black_agent.shortest_path
        #             yellow_input_trajectory = platform.yellow_agent.shortest_path

        #             if platform.black_agent.shortest_path:
        #                 black_input_trajectory = platform.black_agent.shortest_path
        #             else:
        #                 black_input_trajectory = platform.black_agent.input_trajectory
                    
        #             if platform.yellow_agent.shortest_path:
        #                 yellow_input_trajectory = platform.yellow_agent.shortest_path
        #             else:
        #                 yellow_input_trajectory = platform.yellow_agent.input_trajectory

        #             update_trajectories(black_input_trajectory, yellow_input_trajectory, black_ref_trajectory, yellow_ref_trajectory, platform)
        #             # plt.pause(0.1)

        #             # if i == 6:
        #             #     pdb.set_trace()

        #             i += 1

        #         plt.ioff()  # Turn off interactive mode when done
        #         plt.show()
        