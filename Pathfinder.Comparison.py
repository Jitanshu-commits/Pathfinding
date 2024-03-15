# Import Statements                            Updates Pending                          
import random  # For shuffling neighbors in DFS
import pygame  # For graphical visualization
import heapq  # For priority queue implementation in Dijkstra's and A*
from collections import deque  # For implementing the queue in BFS
import tkinter as tk  # For creating GUI for algorithm selection
from tkinter import messagebox  # For displaying message boxes in tkinter GUI
from tkinter import Scale  # For adding the slider

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
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)

GRID_SIZE = 20  #sets the actual range of the algorithms on a grid 
CELL_SIZE = 30
SCREEN_SIZE = (GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE)

# Define Start and End (Grid Range) for algorithms to work on.
START = (0, 0)
END = (GRID_SIZE - 1, GRID_SIZE - 1)

pygame.init()

# Initialize Pygame window
screen = pygame.display.set_mode(SCREEN_SIZE)
pygame.display.set_caption("Pathfinding Visualization")

# Define a set to keep track of modified cells
modified_cells = set()

# Default delay for visualization
DELAY = 50
DELAY_VISITED= 10

# Function to draw the grid
def draw_grid():
    # Draws the grid lines on the Pygame window
    screen.fill(BLACK)  # Fill the screen with black color
    for x in range(0, SCREEN_SIZE[0], CELL_SIZE):
        pygame.draw.line(screen, WHITE, (x, 0), (x, SCREEN_SIZE[1]))
    for y in range(0, SCREEN_SIZE[1], CELL_SIZE):
        pygame.draw.line(screen, WHITE, (0, y), (SCREEN_SIZE[0], y))
    pygame.display.flip()  # Update the display

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
        pygame.time.wait(DELAY)      #User can change wait time to 0 for the real comparison of time taken by each Algorithm.

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
        # Update the UI periodically
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                paused = not paused    

         if paused:
            continue
    
        (cost, current, path) = heapq.heappop(heap)     # Pop the minimum cost node from the heap

        if current in visited:
            continue                       # Skip this node if already visited 
 
        visited.add(current)               # Mark the current node as visited
        if current not in (start, end) and current not in obstacles:
            draw_cell(GREEN, current)      # Draw the cell if it's not the start or end
            # Adjust the delay for visited cells
            pygame.time.wait(DELAY_VISITED)

        if current == end:
            visualize_path(path, YELLOW)
            return

        for neighbor in neighbors(current):                                       # Expand the current node by considering its neighbors
            if neighbor not in visited and neighbor not in obstacles:
                heapq.heappush(heap, (cost + 1, neighbor, path + [current]))      # Add the neighbor to the heap with updated cost and path

        pygame.display.flip()

        # Adjust the delay to control the visualization speed for other updates
        pygame.time.wait(DELAY)
        
    # If the loop completes without finding the end node, display a message
    messagebox.showinfo("Dijkstra's", "End node not reachable!")    

# A* algorithm
def astar(start, end, obstacles):
    # A* algorithm for finding the shortest path on a grid
    heap = [(0, start, [])]
    visited = set()

    while heap:
         # Update the UI periodically
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                paused = not paused    

         if paused:
            continue

        (cost, current, path) = heapq.heappop(heap)

        if current in visited:
            continue

        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(BLUE, current)
            # Adjust the delay for visited cells
            pygame.time.wait(DELAY_VISITED)

        if current == end:
            visualize_path(path, YELLOW)
            return

        for neighbor in neighbors(current):
            if neighbor not in visited and neighbor not in obstacles:
                heapq.heappush(heap, (cost + 1 + heuristic(neighbor, end), neighbor, path + [current]))

        pygame.display.flip()

        # Adjust the delay to control the visualization speed
        pygame.time.wait(DELAY)
        
    # If the loop completes without finding the end node, display a message
    messagebox.showinfo("A*", "End node not reachable!")
    
# DFS algorithm
def dfs(start, end, obstacles):
    # Depth-First Search algorithm for finding a path on a grid
    stack = [(start, [])]
    visited = set()

    while stack:
        # Update the UI periodically
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return
                
        current, path = stack.pop()

        if current in visited:
            continue

        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(BLUE, current)
            # Adjust the delay for visited cells
            pygame.time.wait(DELAY_VISITED)

        if current == end:
            visualize_path(path, YELLOW)
            return

        neighbors_list = neighbors(current)
        random.shuffle(neighbors_list)  # Randomize the order of neighbors for variety

        for neighbor in neighbors_list:
            if neighbor not in visited and neighbor not in obstacles:
                stack.append((neighbor, path + [current]))

        pygame.display.flip()

        # Adjust the delay to control the visualization speed
        pygame.time.wait(DELAY)
        
    # If the loop completes without finding the end node, display a message
    messagebox.showinfo("DFS", "End node not reachable!")
    
# BFS algorithm
def bfs(start, end, obstacles):
    # Breadth-First Search algorithm for finding the shortest path on a grid
    queue = deque([(start, [])])
    visited = set()

    while queue:
         # Update the UI periodically
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return
                
        current, path = queue.popleft()

        if current in visited:
            continue

        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(BLUE, current)
            # Adjust the delay for visited cells
            pygame.time.wait(DELAY_VISITED)

        if current == end:
            visualize_path(path, YELLOW)
            return

        for neighbor in neighbors(current):
            if neighbor not in visited and neighbor not in obstacles:
                # Append neighboring cells to the queue for exploration
                queue.append((neighbor, path + [current]))

        pygame.display.flip()

        # Adjust the delay to control the visualization speed
        pygame.time.wait(DELAY)

     # If the loop completes without finding the end node, display a message
    messagebox.showinfo("BFS", "End node not reachable!")

# Heuristic function for A*
def heuristic(point, goal):     
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])                                                                       
"""
    Heuristic function for A* algorithm. Computes the Manhattan distance.
    :param point: Current point.
    :param goal: Goal point.
    :return: Manhattan distance between the points.  
"""
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
    
# Function to remove an obstacle from the grid
def remove_obstacle(cell, obstacles):
    if cell in obstacles:
        obstacles.remove(cell)
        # Redraw the cell with black color to cover the obstacle
        draw_cell(BLACK, cell)
        
        # Redraw the surrounding lines only for the specific cell
        x, y = cell
        x_pixel = x * CELL_SIZE
        y_pixel = y * CELL_SIZE
        
        # Redraw the horizontal line above the removed obstacle
        if y > 0:
            pygame.draw.line(screen, WHITE, (x_pixel, y_pixel), (x_pixel + CELL_SIZE, y_pixel))
        
        # Redraw the horizontal line below the removed obstacle
        if y < GRID_SIZE - 1:
            pygame.draw.line(screen, WHITE, (x_pixel, y_pixel + CELL_SIZE), (x_pixel + CELL_SIZE, y_pixel + CELL_SIZE))
        
        # Redraw the vertical line to the left of the removed obstacle
        if x > 0:
            pygame.draw.line(screen, WHITE, (x_pixel, y_pixel), (x_pixel, y_pixel + CELL_SIZE))
        
        # Redraw the vertical line to the right of the removed obstacle
        if x < GRID_SIZE - 1:
            pygame.draw.line(screen, WHITE, (x_pixel + CELL_SIZE, y_pixel), (x_pixel + CELL_SIZE, y_pixel + CELL_SIZE))
        
        pygame.display.flip()  # Update the display


# Function to generate a random maze with obstacles
def generate_random_maze(obstacles):
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if random.random() < 0.3:  # Adjust the probability as desired
                cell = (i, j)
                if cell not in (START, END):
                    obstacles.add(cell)
                    draw_cell(GRAY, cell)
                    
# Function to create menu using tkinter
def create_menu():
    root = tk.Tk()
    root.title("Algorithm Selection")

    maze_var = tk.IntVar()
    maze_checkbox = tk.Checkbutton(root, text="Maze", variable=maze_var)
    maze_checkbox.pack()

    # Adding slider for delay
    delay_label = tk.Label(root, text="Visualization Delay:")
    delay_label.pack()
    delay_slider = Scale(root, from_=0, to=200, orient=tk.HORIZONTAL, length=200)
    delay_slider.set(DELAY)  # Set default value
    delay_slider.pack()

    def select_algorithm(algorithm):
        global DELAY
        DELAY = delay_slider.get()  # Update the delay value
        root.destroy()  # Close the menu
        run_algorithm(algorithm, maze_var.get()) # Run the selected algorithm, pass maze_var value
    # Add labels and buttons for each algorithm option
    tk.Label(root, text="Select Algorithm:").pack()
    tk.Button(root, text="A*", command=lambda: select_algorithm(ASTAR)).pack()
    tk.Button(root, text="Dijkstra's", command=lambda: select_algorithm(DIJKSTRA)).pack()
    tk.Button(root, text="BFS", command=lambda: select_algorithm(BFS)).pack()
    tk.Button(root, text="DFS", command=lambda: select_algorithm(DFS)).pack()

    root.mainloop() # Start the tkinter event loop

# Function to run the selected algorithm
def run_algorithm(selected_algorithm, create_maze):
    reset()

    obstacles = set()

    if create_maze:
        generate_random_maze(obstacles)  # Generate maze before setting start and end points
#Main(engine) code
    running = True
    start_set = False
    end_set = False
    start = START
    end = END
    drawing_obstacle = False

    while running: # Inside the pygame event handling loop
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False # Exit the loop and close the program if the user quits
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE: # Run the selected algorithm when the space key is pressed
                    if start_set and end_set:
                        if selected_algorithm == DIJKSTRA:    # Measure the time taken by Dijkstra's algorithm and print it                  
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
                    else:
                        messagebox.showerror("Error", "Start and/or end points are not set.")           
                elif event.key == pygame.K_c:   # If C key is pressed, clear the grid and reset
                    start_set = False
                    end_set = False
                    start = START
                    end = END
                    obstacles = set()
                    reset()
                elif event.key == pygame.K_ESCAPE:
                    create_menu()  # Show the algorithm selection menu
                elif event.key == pygame.K_1:  # Shortcut for A*
                    selected_algorithm = ASTAR
                elif event.key == pygame.K_2:  # Shortcut for Dijkstra's
                    selected_algorithm = DIJKSTRA
                elif event.key == pygame.K_3:  # Shortcut for BFS
                    selected_algorithm = BFS
                elif event.key == pygame.K_4:  # Shortcut for DFS
                    selected_algorithm = DFS                        
            elif event.type == pygame.MOUSEBUTTONDOWN:   # Handle mouse button down events   
                pos = pygame.mouse.get_pos()
                cell_pos = (pos[0] // CELL_SIZE, pos[1] // CELL_SIZE)
                if pygame.mouse.get_pressed()[0]:  # Left mouse button
                    if not start_set:  # If start is not set, set start position
                        start = cell_pos
                        draw_cell(RED, start)
                        start_set = True
                    elif not end_set and cell_pos != start:  # If end is not set and the clicked position is not the same as start, set end position
                        end = cell_pos
                        draw_cell(RED, end)
                        end_set = True
                    else:
                        # Mark the cell as an obstacle
                        obstacles.add(cell_pos)
                        draw_cell(GRAY, cell_pos)
                        drawing_obstacle = True
                elif pygame.mouse.get_pressed()[2]:  # Right mouse button
                    remove_obstacle(cell_pos, obstacles)
            elif event.type == pygame.MOUSEMOTION and drawing_obstacle:
                # If the mouse is moved while the obstacle button is held, add obstacles continuously
                pos = pygame.mouse.get_pos()
                cell_pos = (pos[0] // CELL_SIZE, pos[1] // CELL_SIZE)
                obstacles.add(cell_pos)
                draw_cell(GRAY, cell_pos)
            elif event.type == pygame.MOUSEBUTTONUP:  # Handle mouse button up events, stop drawing obstacles
                drawing_obstacle = False

        pygame.display.flip()

    pygame.quit()

def main():
    create_menu()

if __name__ == "__main__":
    main()
#Fixed object removal
#Added a slider for visualization delay
