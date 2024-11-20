import pdb
import asyncio
import json
import numpy as np
from agent import Agent
from agent_color import AgentColor
from constants import *

class CollisionException(Exception):
    def __init__(self):
        self.message = "\n********\nALERT: COLLISION DETECTED!!!\n********"
        super().__init__(self.message)

class Platform:
    def __init__(self, ipc_client=None): 
        if OPERATION_MODE == "LIVE" and ipc_client == None:
            raise "unassigned object tracking server."
        
        self.ipc_client = ipc_client
        self.generate_meshgrids()
        self.generate_coil_positions()
        self.generate_adjacency_matrix()
        self.create_agents()
        self.current_control_iteration = 0
        self.deactivated_positions = []

        self.ttc, self.prior_ttc = np.inf, np.inf

    def alert_if_collision(self):
        # try:
        if np.linalg.norm(self.black_agent.position - self.yellow_agent.position) <= FIELD_RANGE:
            raise CollisionException
        # except CollisionException as e:
        #     print(e.message)
        #     pdb.set_trace()
    
    def reset_interference_parameters(self):
        for a in self.agents:
            a.motion_plan_updated_at_platform_level = False
            a.adjacency_matrix = self.initial_adjacency_matrix.copy()
        
        self.deactivated_positions = []

    async def advance_agents(self):
        for a in self.agents:
            print(a.color, self.idx_to_grid(a.input_trajectory[self.current_control_iteration+1]))
        
        await asyncio.gather(*[a.advance() for a in self.agents])

    def update_agent_positions(self):
        '''
            in cartesian units wrt a centered origin
        '''

        if OPERATION_MODE == "LIVE":
            messages = self.ipc_client.xrevrange(POSITIONS_STREAM, count=1)
            if messages:
                _, message = messages[0]
                for agent in self.agents:
                    color = f"{agent.color.value}".encode('utf-8')
                    payload = json.loads(message[color].decode())
                    agent.update_position(payload)

        elif OPERATION_MODE == "SIMULATION":
            self.simulate_position_readings()

    def create_agents(self):
        self.black_agent = Agent(self, AgentColor.BLACK)
        self.yellow_agent = Agent(self, AgentColor.YELLOW)
        self.agents = [self.black_agent, self.yellow_agent]

    ## multi-agent Dijkstra ## 
    def perform_halting_collision_avoidance(self):
        for a in self.agents:
            a.halt_for_interference = False

        if any(a.is_undetected() for a in self.agents):
            return
        
        if all(a.is_close_to_reference() for a in self.agents):
            return
        
        i = self.current_control_iteration

        primary, secondary = self.prioritized_agents()
        if np.linalg.norm(primary.position - secondary.position) <= INTERFERENCE_RANGE:
            primary.halt_for_interference = True
            primary_position_idx = self.grid_to_idx(*primary.position)

            secondary.deactivate_positions_within_radius(primary_position_idx)
            secondary.update_motion_plan(secondary.single_agent_shortest_path())
            secondary.motion_plan_updated_at_platform_level = True
        else:
            primary.halt_for_interference = False

    def prioritized_agents(self):
        if self.yellow_agent.is_close_to_reference() and not self.black_agent.is_close_to_reference():
            self.yellow_agent.is_primary = True
            self.black_agent.is_primary = False
            return self.yellow_agent, self.black_agent
        elif self.black_agent.is_close_to_reference() and not self.yellow_agent.is_close_to_reference():
            self.black_agent.is_primary = True
            self.yellow_agent.is_primary = False
            return self.black_agent, self.yellow_agent
        else:
            self.yellow_agent.is_primary = True
            self.black_agent.is_primary = False
            return self.yellow_agent, self.black_agent

    ## steering control functions ##
    def perform_steering_collision_avoidance(self):
        print("PERFORMING STEERING AVOIDANCE...")
        if self.current_control_iteration == 0:
            return

        if any(a.is_undetected() for a in self.agents):
            print("atleast one agent is undetected... ignoring steering collision avoidance.")
            return
                
        if all(agent.is_not_moving() for agent in self.agents):
            return
        
        i = self.current_control_iteration
        self.ego, self.obs = self.black_agent, self.yellow_agent
        ref = self.idx_to_grid(self.ego.ref_trajectory[i])

        rho = round(np.linalg.norm(self.ego.position - self.obs.position), 3)

        self.los = np.arctan2((self.obs.position[1] - self.ego.position[1]), (self.obs.position[0] - self.ego.position[0]))
        psi = np.arctan2((ref[1] - self.obs.position[1]), (ref[0] - self.obs.position[0]))
        self.eta = round((np.pi/2) + self.los - psi, 3)

        ego_x_dot, ego_y_dot, obs_x_dot, obs_y_dot = self.calc_relative_velocities()
        rho_dot = round((abs((obs_x_dot - ego_x_dot)) + abs((obs_y_dot - ego_y_dot))) / rho, 3)
        print("rho: ",rho, "rho_dot: ", rho_dot)
        # print("velocities: ", ego_x_dot, ego_y_dot, obs_x_dot, obs_y_dot)
        self.ttc = round(abs(rho / rho_dot), 3)

        print("eta: ", self.eta, "ttc: ", self.ttc)
        self.evade_by_steering()

        self.prior_ttc = self.ttc

    def evade_by_steering(self):
        # print("ttc: ", self.ttc, "prior_ttc: ", self.prior_ttc)
        # print("ttc is infinity: ", np.isinf(self.ttc), "ttc is unchanged: ", self.ttc == self.prior_ttc)
        if np.isinf(self.ttc) or self.ttc >= self.prior_ttc:
            print(f"not approaching obstacle...")
            return
    
        
        psi_ca_dot = ALPHA_STEERING * self.eta * np.exp(-BETA_STEERING * self.ttc)
        print("approaching obstacle...steering angle rate: ", psi_ca_dot)

        print(self.ego.input_trajectory[0:15])

        diagonal_left, perpendicular_left = self.find_evasive_position_candidates()

        if psi_ca_dot > 0.5 and psi_ca_dot < 0.9:
            print(f"go diagonal... {diagonal_left}")
            input_step = self.current_control_iteration + 1
            self.ego.input_trajectory[input_step] = diagonal_left
            self.ego.motion_plan_updated_at_platform_level = True
        elif psi_ca_dot >= 0.9:
            print(f"go perpendicular...{perpendicular_left}")
            input_step = self.current_control_iteration + 1
            self.ego.input_trajectory[input_step] = perpendicular_left
            self.ego.motion_plan_updated_at_platform_level = True

    def find_evasive_position_candidates(self):
        diagonal_left_angle = self.los + (np.pi / 4)
        perpendicular_left_angle = self.los + (np.pi / 2)

        diagonal_left = np.array([
            max(0, min(14, int(round(self.ego.position[0] + np.cos(diagonal_left_angle))))),
            max(0, min(14, int(round(self.ego.position[1] + np.sin(diagonal_left_angle)))))
        ])

        # diagonal_left = ([
        #     max(0, min(14, diagonal_left[0])),
        #     max(0, min(14, diagonal_left[1]))
        # ])

        perpendicular_left = np.array([
            max(0, min(14, int(round(self.ego.position[0] + np.cos(perpendicular_left_angle))))),
            max(0, min(14, int(round(self.ego.position[1] + np.sin(perpendicular_left_angle)))))
        ])

        # perpendicular_left = ([
        #     max(0, min(14, perpendicular_left[0])),
        #     max(0, min(14, perpendicular_left[1]))
        # ])

        return self.grid_to_idx(*diagonal_left), self.grid_to_idx(*perpendicular_left)
        
    def calc_relative_velocities(self):
        if self.ego.is_not_moving():
            ego_x_dot = 0
            ego_y_dot = 0
        else:
            ego_x_dot = (self.ego.position[0] - self.ego.prior_position[0]) / DEFAULT_ACTUATION_DURATION
            ego_y_dot = (self.ego.position[1] - self.ego.prior_position[1]) / DEFAULT_ACTUATION_DURATION
        
        if self.obs.is_not_moving():
            obs_x_dot = 0
            obs_y_dot = 0
        else:
            obs_x_dot = (self.obs.position[0] - self.obs.prior_position[0]) / DEFAULT_ACTUATION_DURATION
            obs_y_dot = (self.obs.position[1] - self.obs.prior_position[1]) / DEFAULT_ACTUATION_DURATION

        return ego_x_dot, ego_y_dot, obs_x_dot, obs_y_dot
    
    ## initializer functions ##
    def generate_meshgrids(self):
        self.grid_x, self.grid_y = np.meshgrid(np.arange(GRID_WIDTH), np.arange(GRID_WIDTH))

        x_lower, x_upper = -(GRID_WIDTH - 1) / 2, (GRID_WIDTH - 1) / 2
        x_range = np.linspace(x_lower, x_upper, GRID_WIDTH) * COIL_SPACING

        y_lower, y_upper = -(GRID_WIDTH - 1) / 2, (GRID_WIDTH - 1) / 2
        y_range = np.linspace(y_upper, y_lower, GRID_WIDTH) * COIL_SPACING
        self.cartesian_grid_x, self.cartesian_grid_y = np.meshgrid(x_range, y_range)
    
    def generate_coil_positions(self):
        """
        the coil_positions grid is only used to convert the raw visually sensed position 
        into the corresponding grid coordinate. all other operations (including actuation) should
        be performed in terms of grid coordinates.
        """
        coil_positions = [[None for _ in range(GRID_WIDTH)] for _ in range(GRID_WIDTH)]
        for i in range(GRID_WIDTH):
            for j in range(GRID_WIDTH):
                coil_positions[i][j] = np.array([self.cartesian_grid_x[i, j], self.cartesian_grid_y[i, j]])

        self.coil_positions = [pos for row in coil_positions for pos in row]

    def generate_adjacency_matrix(self):
        num_coils = GRID_WIDTH * GRID_WIDTH
        grid_shape = (GRID_WIDTH, GRID_WIDTH)
        A = np.full((num_coils, num_coils), INVALIDATED_NODE_WEIGHT)

        for i in range(GRID_WIDTH):
            for j in range(GRID_WIDTH):
                current_idx = np.ravel_multi_index((i, j), grid_shape)
                neighbors = np.array([
                    [i, j - 1],
                    [i, j + 1],
                    [i - 1, j],
                    [i + 1, j],
                    [i - 1, j - 1],
                    [i - 1, j + 1],
                    [i + 1, j - 1],
                    [i + 1, j + 1],
                ])

                for n_i, n_j in neighbors:
                    if 0 <= n_i < GRID_WIDTH and 0 <= n_j < GRID_WIDTH:
                        neighbor_index = np.ravel_multi_index((int(n_i), int(n_j)), grid_shape)
                        distance = np.linalg.norm(np.array([i, j]) - np.array([n_i, n_j]))
                        A[current_idx, neighbor_index] = distance
                        A[neighbor_index, current_idx] = distance

                A[current_idx, current_idx] = 0

        self.initial_adjacency_matrix = A
    
    ## helper functions ###

    def idx_to_grid(self, idx):
        row = idx // GRID_WIDTH
        col = idx % GRID_WIDTH
        return row, col

    def grid_to_idx(self, row, col):
        return (row * GRID_WIDTH) + col
    
    def cartesian_to_grid(self, x, y):
        x_upper_left = int(7 - y)
        y_upper_left = int(x + 7)
        return np.array([x_upper_left, y_upper_left])
    
    def grid_to_cartesian(self, x, y):
        # Convert coordinates from the upper left origin to the centered origin

        center_offset = (GRID_WIDTH - 1) // 2
        x_centered = y - center_offset
        y_centered = center_offset - x
        return np.array([x_centered, y_centered])
    
    def simulate_position_readings(self):
        if self.current_control_iteration == 0:
            return 
        
        for a in self.agents:
            a.update_position(a.simulated_position_at_end_of_prior_iteration)