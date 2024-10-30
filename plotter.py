import numpy as np
import matplotlib.pyplot as plt
from constants import *
# import pdb

class Plotter:
    def __init__(self, platform):
        self.platform = platform
        self.fig, self.ax = plt.subplots()
        self.create_grid()

    def create_grid(self):
        '''initialize a 15x15 grid with the origin at the top-left'''
        grid_y, grid_x = np.meshgrid(np.arange(GRID_WIDTH), np.arange(GRID_WIDTH))
        self.flat_grid_x = grid_x.flatten()
        self.flat_grid_y = grid_y.flatten()

        self.ax.scatter(grid_x, grid_y, color='gray', s=10)  # Plot grid points
        self.ax.set_xlim(-1, GRID_WIDTH)
        self.ax.set_ylim(GRID_WIDTH, -1)  # Set origin to the top-left
        self.ax.set_xticks(np.arange(0, GRID_WIDTH))
        self.ax.set_yticks(np.arange(0, GRID_WIDTH))
        self.ax.set_aspect('equal')
        plt.ion()
        plt.show()

    def update_plot(self):
        # re-render grid
        self.ax.cla()
        self.create_grid()  

        self.plot_current_agent_positions()
        self.plot_deactivated_positions()
        self.plot_reference_trajectories()
        self.plot_input_trajectories()

        # refresh the plot display
        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()

    def plot_trajectory(self, trajectory, color, label):
        i = self.platform.current_control_iteration
        y_coords, x_coords = np.unravel_index(trajectory[i:], (GRID_WIDTH, GRID_WIDTH))
        self.ax.plot(x_coords, y_coords, 'o-', color=color, label=label)

    def plot_current_agent_positions(self):
        yellow_row, yellow_col = self.platform.yellow_agent.position[0], self.platform.yellow_agent.position[1]    
        black_row, black_col = self.platform.black_agent.position[0], self.platform.black_agent.position[1]    
        self.ax.plot(yellow_col, yellow_row, 'o', color='orange', markersize=15, label='hardcoded')
        self.ax.plot(black_col, black_row, 'o', color='black', markersize=15, label='hardcoded')

    def plot_reference_trajectories(self):
        self.plot_trajectory(self.platform.black_agent.ref_trajectory, color='purple', label='Black Ref Trajectory')
        self.plot_trajectory(self.platform.yellow_agent.ref_trajectory, color='red', label='Yellow Ref Trajectory')

    def plot_input_trajectories(self):
        self.plot_trajectory(self.platform.black_agent.input_trajectory, color='black', label='Black Trajectory')
        self.plot_trajectory(self.platform.yellow_agent.input_trajectory, color='orange', label='Yellow Trajectory')

    def plot_deactivated_positions(self):
        deactivated_indices = [np.ravel_multi_index((y, x), (GRID_WIDTH, GRID_WIDTH)) for x, y in self.platform.deactivated_positions]
        self.deactivated_x = self.flat_grid_x[deactivated_indices]
        self.deactivated_y = self.flat_grid_y[deactivated_indices]
        self.ax.scatter(self.deactivated_x, self.deactivated_y, color='red', s=50, marker='x', label='Deactivated Positions')