import pandas as pd
import matplotlib.pyplot as plt

OUT_OF_RANGE = -1000000
# data = pd.read_csv('final_data/time_series/concentric_circles_halting_155349.csv')
# data = pd.read_csv('final_data/time_series/static_obstacle_steering_avoidance_122338.csv')
# data = pd.read_csv('final_data/time_series/dynamic_obstacle_steering_avoidance_124244.csv')
data = pd.read_csv('final_data/time_series/x_shape_halting_172112_stripped_53p5_to_55.csv')

## strip out datapoints during perturbation
# data = pd.read_csv('final_data/time_series/x_shape_halting_filtered.csv')
# data.loc[(data['timestamp'] >= 53.5) & (data['timestamp'] <= 55), ['black_x', 'black_y', 'yellow_x', 'yellow_y']] = OUT_OF_RANGE
# data.to_csv('x_shape_halting_filtered_53p5_to_55.csv', index=False)
# print(data[(data['timestamp'] >= 53.5) & (data['timestamp'] <= 55)])

filtered_data = data[
    (data['black_x']  != OUT_OF_RANGE) &
    (data['black_y']  != OUT_OF_RANGE) &
    (data['yellow_x'] != OUT_OF_RANGE) &
    (data['yellow_y'] != OUT_OF_RANGE)
]

# Debug: Print a summary of the filtered dataset
print(filtered_data.describe())
print(filtered_data.head())

# Extract coordinates for black and yellow objects
black_x = filtered_data['black_x']
black_y = filtered_data['black_y']
yellow_x = filtered_data['yellow_x']
yellow_y = filtered_data['yellow_y']

# Scatter plot of positions
plt.figure(figsize=(8, 8))
plt.scatter(black_x, black_y, label='Black Agent', color='black', alpha=0.7)
plt.scatter(yellow_x, yellow_y, label='Yellow Agent', color='orange', alpha=0.7)

# Adjust axis limits dynamically with a margin for zooming out
x_min = filtered_data[['black_x', 'yellow_x']].min().min()
x_max = filtered_data[['black_x', 'yellow_x']].max().max()
y_min = filtered_data[['black_y', 'yellow_y']].min().min()
y_max = filtered_data[['black_y', 'yellow_y']].max().max()

# Add a zoom-out margin (e.g., 20% of the range)
x_margin = 0.9 * (x_max - x_min)
y_margin = 0.9 * (y_max - y_min)

plt.xlim(x_min - x_margin, x_max + x_margin)
plt.ylim(y_min - y_margin, y_max + y_margin)

# Add labels and legend
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Experiment 2: Halting Avoidance of a Dynamic Obstacle (with Added Perturbation)')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()

# # data = pd.read_csv('final_data/time_series/concentric_circles_halting_155349.csv')