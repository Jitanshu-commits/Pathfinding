import pygame
import heapq

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
pygame.display.set_caption("Dijkstra and A* Pathfinding Visualization")

# Function to draw the grid
def draw_grid():
    for x in range(0, SCREEN_SIZE[0], CELL_SIZE):
        pygame.draw.line(screen, WHITE, (x, 0), (x, SCREEN_SIZE[1]))
    for y in range(0, SCREEN_SIZE[1], CELL_SIZE):
        pygame.draw.line(screen, WHITE, (0, y), (SCREEN_SIZE[0], y))

# Function to draw a colored cell at a specific position
def draw_cell(color, position):
    pygame.draw.rect(screen, color, (position[0] * CELL_SIZE, position[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

# Function to visualize the path with a delay
def visualize_path(path, color):
    for cell in path:
        draw_cell(color, cell)
        pygame.display.flip()
        pygame.time.wait(75)      #User can change wait time to 0 for the real comparison of time taken by each Algorithm.

# Reset the screen, draw the grid, and update the display
def reset():
    screen.fill(BLACK)
    draw_grid()
    pygame.display.flip()

# Dijkstra's algorithm
def dijkstra(start, end, obstacles):
    heap = [(0, start, [])]  # Priority queue for exploring cells
    visited = set()          # Set to keep track of visited cells

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
    heap = [(0, start, [])]   # Priority queue for exploring cells
    visited = set()           # Set to keep track of visited cells

    while heap:
        (cost, current, path) = heapq.heappop(heap)

        if current in visited:
            continue
        # Check if the current cell is not the start or end, and it's not an obstacle
        visited.add(current)
        if current not in (start, end) and current not in obstacles:
            draw_cell(BLUE, current)
        # Check if the current cell is the end point
        if current == end:
            # Visualize the path with a yellow color
            visualize_path(path, YELLOW)
            return
        # Explore neighbors
        for neighbor in neighbors(current):
            if neighbor not in visited and neighbor not in obstacles:   # Check if the neighbor is not visited and is not an obstacle
                heapq.heappush(heap, (cost + 1 + heuristic(neighbor, end), neighbor, path + [current]))# Add the neighbor to the priority queue with an updated cost and path

        pygame.display.flip()

# Heuristic function for A*
def heuristic(point, goal):
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

# Function to get valid neighbors for a cell
def neighbors(cell):
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

# Main function
def main():
    reset()

    running = True
    start_set = False
    end_set = False
    start = START
    end = END
    obstacles = set()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if start_set and end_set:
                        # Run Dijkstra's and A* simultaneously
                        dijkstra_path = []
                        astar_path = []

                        dijkstra_start_time = pygame.time.get_ticks()
                        dijkstra(start, end, obstacles)
                        dijkstra_end_time = pygame.time.get_ticks()
                        print(f"Dijkstra's Algorithm Time: {dijkstra_end_time - dijkstra_start_time} ms")

                        astar_start_time = pygame.time.get_ticks()
                        astar(start, end, obstacles)
                        astar_end_time = pygame.time.get_ticks()
                        print(f"A* Algorithm Time: {astar_end_time - astar_start_time} ms")
                elif event.key == pygame.K_c:
                    # Reset the environment
                    start_set = False
                    end_set = False
                    start = START
                    end = END
                    obstacles = set()
                    reset()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                cell_pos = (pos[0] // CELL_SIZE, pos[1] // CELL_SIZE)
                # Check if the starting point is not set
                if not start_set:
                    # Set the starting point
                    start = cell_pos
                    draw_cell(RED, start)
                    start_set = True
                # Check if the ending point is not set and the current mouse position is different from the starting point
                elif not end_set and cell_pos != start:
                    # Set the ending point
                    end = cell_pos
                    draw_cell(RED, end)
                    end_set = True
                else:
                    # Mark the cell as an obstacle
                    obstacles.add(cell_pos)
                    draw_cell(GRAY, cell_pos)
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
