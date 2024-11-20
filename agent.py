import asyncio
import math
import numpy as np
import networkx as nx
import pdb
import time
from agent_color import AgentColor
from constants import *

class Agent:
    _actuator = None

    @classmethod
    def set_actuator(cls, actuator):
        cls._actuator = actuator

    def __init__(self, platform, color: AgentColor):
        try:            
            self.platform = platform
            self.color = color
            self.position = np.array([])
            self.is_primary = None

            if OPERATION_MODE == "LIVE":
                self.position = np.array([OUT_OF_RANGE, OUT_OF_RANGE])
            
            if color == AgentColor.BLACK:
                self.orbit = BLACK_ORBIT
                if OPERATION_MODE == "SIMULATION":
                    self.update_position(SIMULATED_INITIAL_BLACK_POSITION)

            elif color == AgentColor.YELLOW:
                self.orbit = YELLOW_ORBIT
                if OPERATION_MODE == "SIMULATION":
                    self.update_position(SIMULATED_INITIAL_YELLOW_POSITION)

            self.ref_trajectory = np.tile(self.orbit, NUM_SAMPLES)
            self.input_trajectory = self.ref_trajectory.copy()
            self.adjacency_matrix = self.platform.initial_adjacency_matrix.copy()
            self.shortest_path = None
            self.halt_for_interference = False

            if OPERATION_MODE == "SIMULATION":
                self.simulated_position_at_end_of_prior_iteration = self.position
                
        except AttributeError:
            raise AttributeError(f"The {color} agent instance failed to initialize successfully.")
            
    # async def advance(self):
    #     if self.is_undetected():
    #         return

    #     if self.halt_for_interference:
    #         print(f"{self.color}: halting...")
    #         return

    #     i = self.platform.current_control_iteration
    #     if i < len(self.input_trajectory):
    #         ref_position = self.platform.idx_to_grid(self.ref_trajectory[i])
            
    #         error = np.linalg.norm(np.array(self.position) - np.array(ref_position))

    #         if error > FIELD_RANGE:
    #             if not self.motion_plan_updated_at_platform_level:
    #                 shortest_path = self.single_agent_shortest_path()
    #                 self.update_motion_plan(shortest_path[:3])

    #             await self.__actuate(self.input_trajectory[i])
    #             await self.__actuate(self.input_trajectory[i + 1])
    #         else:
    #             if not self.motion_plan_updated_at_platform_level:
    #                 if self.input_trajectory[i] != self.ref_trajectory[i]:
    #                     self.update_motion_plan([self.ref_trajectory[i]])

    #             await self.__actuate(self.input_trajectory[i])

    def update_motion_plan(self, inputs):
        i = self.platform.current_control_iteration
        for s, step in enumerate(inputs):
            input_step = i + s
            if input_step < len(self.input_trajectory):
                self.input_trajectory[input_step] = step

    # steering control: working on sim
    async def advance(self):
        if self.is_undetected():
            return
        
        if(self.halt_for_interference):
            print(f"{self.color}: halting...")
            return

        i = self.platform.current_control_iteration
        if i < len(self.input_trajectory):
            ref_position = self.platform.idx_to_grid(self.ref_trajectory[i])

            error = np.linalg.norm(np.array(self.position) - np.array(ref_position))
            if error > FIELD_RANGE:
                if not self.motion_plan_updated_at_platform_level:
                    shortest_path = self.single_agent_shortest_path()
                    self.update_motion_plan(shortest_path[:3])              # set more steps (e.g., 4) to make transition smoother

                await self.__actuate(self.input_trajectory[i+1])
                await self.__actuate(self.input_trajectory[i+2])
            else:
                await self.__actuate(self.input_trajectory[i+1])

    def next_position_in_deactivated_zone(self):
        next_pos = self.input_trajectory[self.platform.current_control_iteration]
        deactivated_positions = [self.platform.grid_to_idx(*dp) for dp in self.platform.deactivated_positions]
        return next_pos in deactivated_positions

    def single_agent_shortest_path(self):
        position_idx = int(self.platform.grid_to_idx(*self.position))
        graph = nx.from_numpy_array(self.adjacency_matrix)
        ref_position_idx = self.ref_trajectory[self.platform.current_control_iteration + 1]
        self.shortest_path = nx.dijkstra_path(graph, position_idx, ref_position_idx)
        return self.shortest_path
    
    def is_secondary(self):
        return not self.is_primary
    
    def is_close_to_reference(self):
        i = self.platform.current_control_iteration
        ref_trajectory_position = np.array(self.platform.idx_to_grid(self.ref_trajectory[i]))
        return np.linalg.norm(ref_trajectory_position - self.position) <= FIELD_RANGE
    
        # if self.color == AgentColor.YELLOW:
        #     pos = self.platform.grid_to_idx(*self.position)
        #     if(pos == 51 and self.ref_trajectory[i] == 69):
        #         pdb.set_trace()
        #     print("yellow: ", self.platform.grid_to_idx(*self.position), self.ref_trajectory[i], "is_close: ", error <= FIELD_RANGE)
        # return error <= FIELD_RANGE
    
    def deactivate_positions_within_radius(self, target_idx):
        target_position = self.platform.idx_to_grid(target_idx)
        for i in range(GRID_WIDTH):
            for j in range(GRID_WIDTH):
                candidate_position = np.array([i, j])
                candidate_idx = self.platform.grid_to_idx(*candidate_position)
                
                if np.linalg.norm(target_position - candidate_position) <= DEACTIVATION_RADIUS:
                    self.platform.deactivated_positions.append(candidate_position)
                    neighbors = np.array([
                        [i, j - 1],
                        [i, j + 1],
                        [i - 1, j],
                        [i + 1, j],
                        [i - 1, j - 1],
                        [i - 1, j + 1],
                        [i + 1, j - 1],
                        [i + 1, j + 1]
                    ])

                    for q, neighbor in enumerate(neighbors):
                        ni, nj = neighbor
                        if 0 <= ni < GRID_WIDTH and 0 <= nj < GRID_WIDTH:
                            neighbor_idx = self.platform.grid_to_idx(ni, nj)
                            self.adjacency_matrix[candidate_idx, neighbor_idx] = INVALIDATED_NODE_WEIGHT
                            self.adjacency_matrix[neighbor_idx, candidate_idx] = INVALIDATED_NODE_WEIGHT

    def get_one_layer_neighbors(self, position_idx):
        """
        Returns the indices of the positions that are one layer (directly adjacent)
        around the given position in the grid.
        """
        neighbors = []
        row, col = self.platform.idx_to_grid(position_idx)

        # Loop through adjacent cells
        for i in range(row - 1, row + 2):
            for j in range(col - 1, col + 2):
                if 0 <= i < GRID_WIDTH and 0 <= j < GRID_WIDTH:
                    neighbors.append(self.platform.grid_to_idx(i, j))

        return neighbors
    
    def is_undetected(self):
        return np.any(self.position <= OUT_OF_RANGE)
    
    def is_not_moving(self):
        return np.array_equal(self.position, self.prior_position)

    def update_position(self, new_position):
        if not np.any(self.position):
            self.prior_position = new_position            
        else:
            self.prior_position = self.position

        self.position = self.__coerce_position(new_position)
        # print(f"{self.color}: new position: {self.position}, new_position: {self.platform.grid_to_idx(*self.position)}")

    async def __actuate(self, new_position_idx):
        if OPERATION_MODE == "LIVE":
            await self._actuator.actuate_single(*self.platform.idx_to_grid(new_position_idx))
        
        elif OPERATION_MODE == "SIMULATION":
            new_position = self.platform.idx_to_grid(new_position_idx)
            self.simulated_position_at_end_of_prior_iteration = self.platform.grid_to_cartesian(*new_position)
            print(f"{self.color} ON: {new_position}")
            print(f"{self.color} OFF: {new_position}")

    # TODO: coerce to the closest grid position
    def __coerce_position(self, measured_position):
        """
        - Coerce the current position to the raw coordinates of the nearest coil if within the coersion threshold.
        - This helps filter for tracking noise and discretizes the measured position to that of the nearest coil.
        """

        #TODO: are we updating to the correct grid position from the cartesian readings?
        if np.any(np.array(measured_position) == OUT_OF_RANGE):
            return np.array([OUT_OF_RANGE, OUT_OF_RANGE])
        
        if OPERATION_MODE == "SIMULATION":
            # print(self.color, "measured position: ", measured_position)
            raw_grid = self.platform.cartesian_to_grid(*np.array(measured_position))
            return raw_grid
        else:
            distances = np.linalg.norm(self.platform.coil_positions - np.array(measured_position), axis=1)
            closest_idx = np.argmin(distances)
            return np.array(self.platform.idx_to_grid(closest_idx))
