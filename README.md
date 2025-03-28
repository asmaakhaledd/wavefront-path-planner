# Wavefront Path Planning

## Project Overview
This project implements a grid-based wavefront path planning algorithm to navigate a 2D maze. The goal is to find the shortest path from a defined start position to a goal, while avoiding obstacles, using wavefront expansion and backtracking the path.

---

## Algorithm Summary
The algorithm performs the following steps:

1. **Load Maze**: Load a maze matrix from `maze.mat` where:
   - `0`: Free space  
   - `1`: Wall/Obstacle  
   - `2`: Goal location

2. **Wavefront Expansion (`wavefront_expansion`)**: Performs a BFS (Breadth-First Search) from the goal to compute a `value_map` indicating distance from the goal.

3. **Extract Path (`extract_path`)**: Traces the shortest path from the start to the goal by moving to the neighboring cell with the smallest value.

4. **Plot Path (`plot_map`)**: Plots the original maze and overlays the found path in red.

---

## Files Included
- `wavefront_planner.py`: Main implementation of the algorithm.  
- `maze.mat`: Input maze environment.  
- `value_map.txt`: Text file output of the computed wavefront values.  
- `trajectory.txt`: List of coordinates representing the shortest path.  
- `trajectory_map.png`: Visual output showing the path in the maze.

---

## How to Run

1. Ensure you have Python installed with the following packages:
```bash
pip install numpy matplotlib scipy
```
Place maze.mat in the same directory as the script.

Run the script:
```bash
python planner.py
```
The outputs value_map.txt, trajectory.txt, and trajectory_map.png will be saved in the same directory.
