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
from collections import deque



# Load the .mat file
data = scipy.io.loadmat('maze.mat')
# Extract the map 
maze_map = data['map']
# Print to verify
print("Maze shape:",maze_map.shape) 
goal_pos = np.argwhere(maze_map == 2)
print("Goal positions found:", goal_pos)


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
    
     # Ensure map is an integer array (fixes potential data type issues)
    map = map.astype(int)
    
     # Find the goal position
    goal_pos = np.argwhere(map == 2)  # Find goal (should return an array of indices)
    if goal_pos.size == 0:
        raise ValueError("No goal (2) found in the map!")
    
    goal_pos = tuple(goal_pos[0])  # Convert to tuple (row, col)

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



# Test the planner, Define the start position
start_row, start_column = 45, 4 

start_time = time.time()

# Run the wavefront planner
value_map, trajectory = planner(maze_map, start_row, start_column)

end_time = time.time()
# Calculate execution time
execution_time = end_time - start_time

print("Execution time: {:.6f} seconds".format(execution_time))

plot_map(maze_map, trajectory)



