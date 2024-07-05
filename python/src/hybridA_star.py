import numpy as np
import matplotlib.pyplot as plt
from heapq import heappop, heappush
from matplotlib.animation import FuncAnimation

# Define the environment
grid_size = (20, 20)
obstacles = [(5, 5), (5, 6), (6, 5), (6, 6), (10, 10), (10, 11), (11, 10), (11, 11)]

# Define the start and goal positions
start = (0, 0, 0)  # (x, y, theta)
goal = (19, 19)

# Define the bicycle model parameters
L = 2.0  # Wheelbase
dt = 0.5  # Time step

# Heuristic function
def heuristic(a, b):
    return np.hypot(b[0] - a[0], b[1] - a[1])

# Check if the position is within the grid and not an obstacle
def is_valid(x, y):
    return 0 <= x < grid_size[0] and 0 <= y < grid_size[1] and (x, y) not in obstacles

# Hybrid A* algorithm
def hybrid_a_star(start, goal):
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    path = []

    while open_set:
        _, current = heappop(open_set)
        x, y, theta = current

        if (x, y) == goal:
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        # Generate possible moves considering kinematic constraints
        for phi in [-np.pi / 4, 0, np.pi / 4]:  # Steering angles
            x_new = x + np.cos(theta) * dt
            y_new = y + np.sin(theta) * dt
            theta_new = theta + (1 / L) * np.tan(phi) * dt

            if is_valid(int(x_new), int(y_new)):
                neighbor = (x_new, y_new, theta_new)
                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        path.append(current)
        yield open_set, path  # Yield the open set and current path for animation

    yield open_set, path  # Final open set and path

# Setup the plot
fig, ax = plt.subplots(figsize=(10, 10))
ax.grid(True)
ax.set_xlim(0, grid_size[0])
ax.set_ylim(0, grid_size[1])

# Plot obstacles
for obs in obstacles:
    ax.plot(obs[0], obs[1], 'ks')

# Plot start and goal
ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

# Initialize plot elements for the open set and path
open_set_points, = ax.plot([], [], 'co', markersize=5, label='Open set')
path_line, = ax.plot([], [], 'b-', label='Path')

# Animation update function
def update(data):
    open_set, path = data
    #if open_set:
    #    open_set_x, open_set_y = zip(*[(node[1][0], node[1][1]) for node in open_set])
    #    open_set_points.set_data(open_set_x, open_set_y)
    if path:
        path_x, path_y = zip(*[(int(p[0]), int(p[1])) for p in path])
        path_line.set_data(path_x, path_y)
    return open_set_points, path_line

# Create the animation
planner = hybrid_a_star(start, goal)
ani = FuncAnimation(fig, update, frames=planner, blit=True, repeat=False)

plt.legend()
plt.show()

