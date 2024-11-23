import numpy as np
import math
'''
    - all values are measured in centimeters [cm] unless inches [in] are explicitly specified in variable name
    - variable notation: "x" represents a value in centimeters. "x_inches" represents a value in inches
    - wherever necessary, units will be specified for each parameter in comments using the [unit] notation (ex. [cm] for centimeters)
    - [#] represents a dimensionless numerical value
'''

########## OPERATION MODE: "SIMULATION" or "LIVE" #############
# OPERATION_MODE = "SIMULATION"
OPERATION_MODE = "LIVE"
########## OPERATION MODES #############

########## AVOIDANCE STRATEGY: "HALTING" or "STEERING" #############
# CONTROL_MODE = "HALTING" 
CONTROL_MODE = "STEERING"

# more aggressive
ALPHA_STEERING = 3
BETA_STEERING = 0.03

# less aggressive
# ALPHA_STEERING = 2
# BETA_STEERING = 0.08

########## CONTROL MODES #############

########## REFERENCE ORBITS AND INITIAL POSITIONS (cartesian coordinates) #############
## exp 1: halting: x shape
# EXPERIMENT_NAME = "x_shape_halting"
# YELLOW_ORBIT = [33, 34, 50, 65, 79, 78, 62, 47]
# BLACK_ORBIT  = [40, 41, 57, 72, 86, 85, 69, 54]
# SIMULATED_INITIAL_BLACK_POSITION = np.array([-2, -5])
# SIMULATED_INITIAL_YELLOW_POSITION  = np.array([1, -3])
# SIMULATED_INITIAL_BLACK_POSITION = np.array([-6, -5])
# SIMULATED_INITIAL_YELLOW_POSITION  = np.array([1, -3])

# SIMULATED_INITIAL_YELLOW_POSITION = np.array([-4, 5])
# SIMULATED_INITIAL_BLACK_POSITION  = np.array([3, 5])
# SIMULATED_INITIAL_YELLOW_POSITION = np.array([-6, -5])
# SIMULATED_INITIAL_BLACK_POSITION  = np.array([1, -3])

## exp 2: halting: concentric circles (initial condition: close to reference trajectories)
# EXPERIMENT_NAME = "concentric_circles_halting"
# YELLOW_ORBIT = [50, 51, 52, 53, 69, 85, 100, 115, 130, 145, 160, 174, 188, 187, 186, 185, 169, 153, 138, 123, 108, 93, 78, 64]
# BLACK_ORBIT  = [96, 97, 113, 128, 142, 141, 125, 110] 
# SIMULATED_INITIAL_YELLOW_POSITION = np.array([-2, 3])
# SIMULATED_INITIAL_BLACK_POSITION  = np.array([0, 0])

## exp 3: steering: static obstacle
# EXPERIMENT_NAME = "static_obstacle_steering_avoidance"
# SIMULATED_INITIAL_YELLOW_POSITION = np.array([-1, 0])
# SIMULATED_INITIAL_BLACK_POSITION  = np.array([3, 5])
# BLACK_ORBIT = [196]
# YELLOW_ORBIT = [111]

# exp 4 steering: intersection dynamic obstacle
EXPERIMENT_NAME = "dynamic_obstacle_steering_avoidance"
SIMULATED_INITIAL_YELLOW_POSITION = np.array([-2, 3])
SIMULATED_INITIAL_BLACK_POSITION  = np.array([5, 6])
BLACK_ORBIT = [182]
YELLOW_ORBIT = [127]

# BLACK_ORBIT = [183]
# YELLOW_ORBIT = [156]

# # exp 4: steering: x-shape dynamic obstacle
# SIMULATED_INITIAL_YELLOW_POSITION = np.array([-7, 7])
# SIMULATED_INITIAL_BLACK_POSITION  = np.array([-7, -7])
# BLACK_ORBIT = [27]
# YELLOW_ORBIT = [224]
########## REFERENCE ORBITS AND INITIAL POSITIONS (cartesian coordinates) #############

########## EXPERIMENT PARAMETERS #############
REF_TRAJECTORY_PERIOD = 200                                                     # total time period [sec]
SAMPLING_PERIOD       = 0.0625                                                  # camera sampling period [sec]
NUM_SAMPLES           = int(np.ceil(REF_TRAJECTORY_PERIOD / SAMPLING_PERIOD))

# platform parameters
GRID_WIDTH            = 15                                                      # grid dimensions for static dipoles [#]
NUM_COILS             = GRID_WIDTH * GRID_WIDTH
COIL_SPACING          = 2.159                                                   # spacing between static dipoles: 2.159 [cm]
COERSION_THRESHOLD_IN = 0.4                                                     # a sampled position within this threshold of a coil could be coerced to coil centroid position [in]
SAMPLING_PERIOD       = 0.1                                                     # time between camera readings [sec]

FIELD_RANGE                = math.sqrt(2)                                       # magnetic force range (discretized to 1 diagonal grid position separation)
OUT_OF_RANGE               = -1000000
COERSION_THRESHOLD         = COERSION_THRESHOLD_IN * 2.54                       # coersion threshold [cm]
DEACTIVATION_RADIUS        = math.sqrt(3)                                       # [# of diagonals]
INTERFERENCE_RANGE         = 2 * DEACTIVATION_RADIUS
SAFE_ZONE_RADIUS           = 1
INVALIDATED_NODE_WEIGHT    = np.inf

# redis parameters
POSITIONS_STREAM = 'stream_positions'

# actuator parameters
DEFAULT_ACTUATION_DURATION = 0.3
DEFAULT_DUTY_CYCLE         = 4095
ACTUATOR_PORT = "/dev/cu.usbmodem21301"

CLUSTER_SIZE = 3
########## EXPERIMENT PARAMETERS #############