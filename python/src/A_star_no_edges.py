import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def heuristic(a, b):
    return np.abs(a[0] - b[0]) + np.abs(a[1] - b[1])

def astar(grid, start, end):
    open_list = []
    closed_list = []

    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list.append(start_node)

    while len(open_list) > 0:
        current_node = min(open_list, key=lambda o: o.f)
        open_list.remove(current_node)
        closed_list.append(current_node.position)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            visualize_final(grid, path, start, end)  # Visualize final path
            return path[::-1]  # Return reversed path

        visualize(grid, start, end, current_node, open_list, closed_list)

        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

        for next in neighbors:
            if next[0] > (len(grid) - 1) or next[0] < 0 or next[1] > (len(grid[len(grid)-1]) - 1) or next[1] < 0:
                continue
            if grid[next[0]][next[1]] != 0:
                continue
            if next in closed_list:
                continue

            neighbor = Node(current_node, next)
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor.position, end_node.position)
            neighbor.f = neighbor.g + neighbor.h

            if add_to_open(open_list, neighbor):
                open_list.append(neighbor)

    return None  # No path found

def add_to_open(open_list, neighbor):
    for node in open_list:
        if neighbor == node and neighbor.g > node.g:
            return False
    return True

def visualize(grid, start, end, current_node, open_list, closed_list):
    plt.figure(figsize=(8, 8))
    plt.title(f"Exploring: {current_node.position}")
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if (row, col) in closed_list:
                plt.scatter(col, row, color='lightgray', s=100)  # Closed nodes
            elif (row, col) == current_node.position:
                plt.scatter(col, row, color='orange', s=100)  # Current node
            elif grid[row][col] == 1:
                plt.scatter(col, row, color='black', s=100)  # Obstacle
            else:
                plt.scatter(col, row, color='white', s=100)  # Empty cells

    for node in open_list:
        plt.scatter(node.position[1], node.position[0], color='green', s=100)  # Open list nodes

    if current_node.parent:
        plt.plot([current_node.position[1], current_node.parent.position[1]],
                 [current_node.position[0], current_node.parent.position[0]], color="blue")  # Path taken

    plt.scatter(start[1], start[0], color='purple', s=200)  # Start node
    plt.scatter(end[1], end[0], color='red', s=200)  # End node
    plt.grid()
    plt.pause(0.1)
    plt.clf()

def visualize_final(grid, path, start, end):
    plt.figure(figsize=(8, 8))
    plt.title("Final Path")
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if (row, col) in path:
                plt.scatter(col, row, color='blue', s=100)  # Path node
            elif grid[row][col] == 1:
                plt.scatter(col, row, color='black', s=100)  # Obstacle
            else:
                plt.scatter(col, row, color='white', s=100)  # Empty cells

    for i in range(len(path) - 1):
        plt.plot([path[i][1], path[i + 1][1]], [path[i][0], path[i + 1][0]], color="yellow")

    plt.scatter(start[1], start[0], color='purple', s=200)  # Start node
    plt.scatter(end[1], end[0], color='red', s=200)  # End node
    plt.grid()
    plt.show()  # This will keep the plot window open

# Example of a grid, start and end positions
grid = np.array([
    [0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1],
    [0, 1, 0, 0, 0]
])
start = (0, 0)
end = (4, 4)

path = astar(grid, start, end)
if not path:
    print("No path found")

