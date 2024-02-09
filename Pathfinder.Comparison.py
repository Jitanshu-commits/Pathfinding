# Import Statements                                                               #Updates Pending
import random  # For shuffling neighbors in DFS
import pygame  # For graphical visualization
import heapq  # For priority queue implementation in Dijkstra's and A*
from collections import deque  # For implementing the queue in BFS
import tkinter as tk
from tkinter import messagebox

# Constants for algorithm selection
ASTAR = 1
DIJKSTRA = 2
BFS = 3
DFS = 4

# Constants
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (128, 128, 128)

GRID_SIZE = 20
CELL_SIZE = 30
SCREEN_SIZE = (GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE)

START = (0, 0)
END = (GRID_SIZE - 1, GRID_SIZE - 1)

pygame.init()

# Initialize Pygame window
screen = pygame.display.set_mode(SCREEN_SIZE)
pygame.display.set_caption("Pathfinding Visualization")

# Function to draw the grid
def draw_grid():
    # Draws the grid lines on the Pygame window
    for x in range(0, SCREEN_SIZE[0], CELL_SIZE):
        pygame.draw.line(screen, WHITE, (x, 0), (x, SCREEN_SIZE[1]))
    for y in range(0, SCREEN_SIZE[1], CELL_SIZE):
        pygame.draw.line(screen, WHITE, (0, y), (SCREEN_SIZE[0], y))

# Function to draw a colored cell at a specific position
def draw_cell(color, position):
    # Draws a colored cell on the Pygame window at the specified position
    pygame.draw.rect(screen, color, (position[0] * CELL_SIZE, position[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

# Function to visualize the path with a delay
def visualize_path(path, color):
    # Visualizes the path on the Pygame window with a delay for better visualization
    for cell in path:
        draw_cell(color, cell)
        pygame.display.flip()
        pygame.time.wait(50)      #User can change wait time to 0 for the real comparison of time taken by each Algorithm.

# Reset the screen, draw the grid, and update the display
def reset():
    # Resets the Pygame window by filling it with black color and redrawing the grid
    screen.fill(BLACK)
    draw_grid()
    pygame.display.flip()

# Dijkstra's algorithm
def dijkstra(start, end, obstacles):
    # Dijkstra's algorithm for finding the shortest path on a grid
    heap = [(0, start, [])]
    visited = set()

    while heap:
        (cost, current, path) = heapq.heappop(heap)

        if current in visited:
            continue

        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(GREEN, current)

        if current == end:
            visualize_path(path, YELLOW)
            return

        for neighbor in neighbors(current):
            if neighbor not in visited and neighbor not in obstacles:
                heapq.heappush(heap, (cost + 1, neighbor, path + [current]))

        pygame.display.flip()

# A* algorithm
def astar(start, end, obstacles):
    # A* algorithm for finding the shortest path on a grid
    heap = [(0, start, [])]
    visited = set()

    while heap:
        (cost, current, path) = heapq.heappop(heap)

        if current in visited:
            continue

        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(BLUE, current)

        if current == end:
            visualize_path(path, YELLOW)
            return

        for neighbor in neighbors(current):
            if neighbor not in visited and neighbor not in obstacles:
                heapq.heappush(heap, (cost + 1 + heuristic(neighbor, end), neighbor, path + [current]))

        pygame.display.flip()

# DFS algorithm
def dfs(start, end, obstacles):
    # Depth-First Search algorithm for finding a path on a grid
    stack = [(start, [])]
    visited = set()

    while stack:
        current, path = stack.pop()

        if current in visited:
            continue

        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(BLUE, current)

        if current == end:
            visualize_path(path, YELLOW)
            return

        neighbors_list = neighbors(current)
        random.shuffle(neighbors_list)  # Randomize the order of neighbors for variety

        for neighbor in neighbors_list:
            if neighbor not in visited and neighbor not in obstacles:
                stack.append((neighbor, path + [current]))

        pygame.display.flip()

# BFS algorithm
def bfs(start, end, obstacles):
    # Breadth-First Search algorithm for finding the shortest path on a grid
    queue = deque([(start, [])])
    visited = set()

    while queue:
        current, path = queue.popleft()

        if current in visited:
            continue

        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(BLUE, current)

        if current == end:
            visualize_path(path, YELLOW)
            return

        for neighbor in neighbors(current):
            if neighbor not in visited and neighbor not in obstacles:
                # Append neighboring cells to the queue for exploration
                queue.append((neighbor, path + [current]))

        pygame.display.flip()

# Heuristic function for A*
def heuristic(point, goal):
      """
    Heuristic function for A* algorithm. Computes the Manhattan distance.
    :param point: Current point.
    :param goal: Goal point.
    :return: Manhattan distance between the points.
    """
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

# Function to get valid neighbors for a cell
def neighbors(cell):
    # Returns valid neighbors for a given cell on the grid
    x, y = cell
    valid_neighbors = []
    if x > 0:
        valid_neighbors.append((x - 1, y))
    if x < GRID_SIZE - 1:
        valid_neighbors.append((x + 1, y))
    if y > 0:
        valid_neighbors.append((x, y - 1))
    if y < GRID_SIZE - 1:
        valid_neighbors.append((x, y + 1))
    return valid_neighbors

def create_menu():
    root = tk.Tk()
    root.title("Algorithm Selection")

    def select_algorithm(algorithm):
        root.destroy()  # Close the menu
        run_algorithm(algorithm)

    tk.Label(root, text="Select Algorithm:").pack()

    tk.Button(root, text="A*", command=lambda: select_algorithm(ASTAR)).pack()
    tk.Button(root, text="Dijkstra's", command=lambda: select_algorithm(DIJKSTRA)).pack()
    tk.Button(root, text="BFS", command=lambda: select_algorithm(BFS)).pack()
    tk.Button(root, text="DFS", command=lambda: select_algorithm(DFS)).pack()

    root.mainloop()

def run_algorithm(selected_algorithm):
    reset()

    running = True
    start_set = False
    end_set = False
    start = START
    end = END
    obstacles = set()
    drawing_obstacle = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if start_set and end_set and selected_algorithm is not None:
                        if selected_algorithm == DIJKSTRA:
                            dijkstra_start_time = pygame.time.get_ticks()
                            dijkstra(start, end, obstacles)
                            dijkstra_end_time = pygame.time.get_ticks()
                            print(f"Dijkstra's Algorithm Time: {dijkstra_end_time - dijkstra_start_time} ms")
                        elif selected_algorithm == ASTAR:
                            astar_start_time = pygame.time.get_ticks()
                            astar(start, end, obstacles)
                            astar_end_time = pygame.time.get_ticks()
                            print(f"A* Algorithm Time: {astar_end_time - astar_start_time} ms")
                        elif selected_algorithm == BFS:
                            bfs_start_time = pygame.time.get_ticks()
                            bfs(start, end, obstacles)
                            bfs_end_time = pygame.time.get_ticks()
                            print(f"BFS Algorithm Time: {bfs_end_time - bfs_start_time} ms")
                        elif selected_algorithm == DFS:
                            dfs_start_time = pygame.time.get_ticks()
                            dfs(start, end, obstacles)
                            dfs_end_time = pygame.time.get_ticks()
                            print(f"DFS Algorithm Time: {dfs_end_time - dfs_start_time} ms")
                elif event.key == pygame.K_c:
                    start_set = False
                    end_set = False
                    start = START
                    end = END
                    obstacles = set()
                    reset()
                elif event.key == pygame.K_ESCAPE:
                    create_menu()  # Show the algorithm selection menu
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                cell_pos = (pos[0] // CELL_SIZE, pos[1] // CELL_SIZE)
                if not start_set:
                    start = cell_pos
                    draw_cell(RED, start)
                    start_set = True
                elif not end_set and cell_pos != start:
                    end = cell_pos
                    draw_cell(RED, end)
                    end_set = True
                else:
                    # Mark the cell as an obstacle
                    obstacles.add(cell_pos)
                    draw_cell(GRAY, cell_pos)
                    drawing_obstacle = True
            elif event.type == pygame.MOUSEMOTION and drawing_obstacle:
                # If the mouse is moved while the obstacle button is held, add obstacles continuously
                pos = pygame.mouse.get_pos()
                cell_pos = (pos[0] // CELL_SIZE, pos[1] // CELL_SIZE)
                obstacles.add(cell_pos)
                draw_cell(GRAY, cell_pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                drawing_obstacle = False

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
