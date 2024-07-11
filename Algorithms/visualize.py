import tkinter as tk
from collections import deque

# Define the maze and initialize the flood fill values
maze = [
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0],
    [1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0],
    [0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
    [0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1],
    [0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

flood_fill = [[-1 for _ in range(16)] for _ in range(16)]

# Flood fill algorithm
def flood_fill_algorithm(maze, flood_fill, start):
    rows, cols = len(maze), len(maze[0])
    queue = deque([start])
    flood_fill[start[0]][start[1]] = 0
    
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    while queue:
        x, y = queue.popleft()
        current_distance = flood_fill[x][y]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            if 0 <= nx < rows and 0 <= ny < cols and maze[nx][ny] == 0 and flood_fill[nx][ny] == -1:
                flood_fill[nx][ny] = current_distance + 1
                queue.append((nx, ny))

goal = (7, 7)
flood_fill_algorithm(maze, flood_fill, goal)

# Visualization with tkinter
def draw_grid(canvas, maze, flood_fill):
    cell_size = 30
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            x1, y1 = j * cell_size, i * cell_size
            x2, y2 = x1 + cell_size, y1 + cell_size
            
            if maze[i][j] == 1:
                color = "black"
            else:
                if flood_fill[i][j] == -1:
                    color = "white"
                else:
                    temp =  (hex((255 - flood_fill[i][j])*8)[-2:])[-2:]
                    print(temp)
                    color = f"#{temp}ff00"

            # number on the grid color black
            if i == 7 and j == 7:
                color = "red"
            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
            canvas.create_text(x1 + cell_size // 2, y1 + cell_size // 2, text=flood_fill[i][j], fill="black")
            

# Main tkinter window
root = tk.Tk()
root.title("Micromouse Flood Fill Visualization")

canvas = tk.Canvas(root, width=480, height=480)
canvas.pack()

draw_grid(canvas, maze, flood_fill)

root.mainloop()
