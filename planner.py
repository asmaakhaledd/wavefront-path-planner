# steps:
# ├── Imported Libraries
# ├── wavefront_expansion()  → Generate value map
# ├── extract_path()         → Extract shortest path
# ├── planner()              → Main function combining both
# ├── plot_map()             → Visualization function
# └── __main__ block         → Test case

import numpy as np
import matplotlib.pyplot as plt
import scipy.io
import time
from mazelib.generate.Prims import Prims
from collections import deque



# Load the .mat file
data = scipy.io.loadmat('maze.mat')
# Extract the map 
maze_map = data['map']
# Print to verify
print("Maze shape:",maze_map.shape) 



def wavefront_expansion(grid, goal):
    """
    Computes the value map using wavefront expansion.
    """
    # Initialize the value map
    value_map = np.copy(grid)
    value_map[goal] = 2

    # Initialize the wavefront queue
    wavefront = deque()
    wavefront.append(goal)

    # Wavefront expansion
    # 8-Point Connectivity 
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), 
                  (-1, 1), (1, 1), (1, -1), (-1, -1)]  

    # Wavefront expansion
    while wavefront:
        current = wavefront.popleft()
        for dr, dc in directions:
            nr, nc = current[0] + dr, current[1] + dc
            if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:  # Inside grid
                if grid[nr, nc] == 0 and value_map[nr, nc] == 0:  
                    value_map[nr, nc] = value_map[current] + 1
                    wavefront.append((nr, nc))

    return value_map


def extract_path(value_map, start_row, start_column):
    """
    Extracts one optimal path from the start position to the goal using the given priority order.
    """
    trajectory = [(start_row, start_column)]
    r, c = start_row, start_column
    
    # Priority order: [Upper, Right, Lower, Left, Upper Right, Lower Right, Lower Left, Upper Left]
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1), 
                  (-1, 1), (1, 1), (1, -1), (-1, -1)]  

    while value_map[r, c] != 2:  # Stop when reaching the goal
        min_value = float('inf')
        next_pos = None

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < value_map.shape[0] and 0 <= nc < value_map.shape[1]:
                if 2 <= value_map[nr, nc] < min_value:  # Find the lowest valid value
                    min_value = value_map[nr, nc]
                    next_pos = (nr, nc)

        if next_pos is None:  # No valid path found (should not happen)
            break

        r, c = next_pos
        trajectory.append((r, c))

    return trajectory


def planner(map, start_row, start_column):
    """
    Computes the optimal trajectory using wavefront planning.
    """

    # Ensure map is an integer array 
    map = map.astype(int) 

    # Find the goal position
    goal_pos = []
    for r in range(len(map)):  # Iterate over rows
        for c in range(len(map[0])):  # Iterate over columns
            if map[r][c] == 2:
                goal_pos.append((r, c))  # Store (row, col) tuple

    if not goal_pos:  # If goal_pos is empty, no goal is found
        raise ValueError("No goal (2) found in the map!")
    
    goal_pos = goal_pos[0]  # Take the first goal position

    print("Goal positions found:", goal_pos)

    # Generate the value map
    value_map = wavefront_expansion(map, goal_pos)

    # Extract the shortest path
    trajectory = extract_path(value_map, start_row, start_column)

    np.savetxt("value_map.txt", value_map, fmt='%d')
    np.savetxt("trajectory.txt", trajectory, fmt='%d')

    return value_map, trajectory


def plot_map(map, trajectory):
    """
    Plots the environment map and the computed trajectory.
    """
    fig, ax = plt.subplots()
    ax.imshow(map, cmap='gray_r')

    # Mark trajectory in red
    for (r, c) in trajectory:
        plt.scatter(c, r, color='red')

    plt.title("Wavefront Path Planning")
    plt.savefig("trajectory_map.png")
    plt.show()

#  This function is intended for testing and validation purposes only. 
# It creates a grid-based environment with random obstacles and a randomly placed goal.

# def generate_random_map(rows=30, cols=50, obstacle_ratio=0.2):
#     """
#     Generates a random 2D map for wavefront path planning.
#     """
#     # Create an empty grid initialized with zeros (free space)
#     grid = np.zeros((rows, cols), dtype=int)
#     # Calculate the number of obstacles based on the given ratio
#     num_obstacles = int(rows * cols * obstacle_ratio)

#     # Place obstacles randomly
#     obstacle_indices = np.random.choice(rows * cols, num_obstacles, replace=False)
#     for index in obstacle_indices:
#         r, c = divmod(index, cols) # Convert linear index to 2D index
#         grid[r, c] = 1    # Mark cell as an obstacle

#     # Place a goal somewhere not an obstacle
#     while True:
#         gr, gc = np.random.randint(0, rows), np.random.randint(0, cols)
#         if grid[gr, gc] == 0:
#             grid[gr, gc] = 2  # Mark as goal
#             break

#     return grid

# Test the planner, Define the start position
start_row, start_column = 45, 4 

start_time = time.time()

# Run the wavefront planner
value_map, trajectory = planner(maze_map, start_row, start_column)

# Testing 

# # Generate the test map
# large_map = generate_random_map(40, 80, obstacle_ratio=0.4)
# # Choose a random start position (not an obstacle or goal)
# while True:
#     sr, sc = np.random.randint(0, 30), np.random.randint(0, 50)
#     if large_map[sr, sc] == 0:
#         break
# # Run the planner
# value_map, trajectory = planner(large_map, sr, sc)


end_time = time.time()
# Calculate execution time
execution_time = end_time - start_time

print("Execution time: {:.6f} seconds".format(execution_time))

plot_map(maze_map, trajectory)

# plot_map(large_map, trajectory) #plotting of test map


