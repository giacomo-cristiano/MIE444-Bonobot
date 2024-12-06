
# Load your precomputed matrix with positions, orientations, and LiDAR readings
# Format for each entry in map_lidar_data: [x, y, theta, lidar_readings]
# Original data format as a single dictionary

import socket
import time
from datetime import datetime
import serial
import ast
import random
import numpy as np
import math
import matplotlib.pyplot as plt
import pygame.math as pm  # Assuming you have the 'pygame' library for the Vector2 class
import os

os.chdir(r"C:\Users\User\OneDrive - University of Toronto\Desktop\SIMMER LOCAL")

print("Changed to directory:", os.getcwd())

# Load your precomputed matrix with positions, orientations, and LiDAR readings
# Format for each entry in map_lidar_data: [x, y, theta, lidar_readings]
# Original data format as a single dictionary
import pickle
# Load the dictionary from the pickle file
with open("large_dict.pkl", "rb") as file:
    Mickel_Table = pickle.load(file)

# Initialize the figure and axes outside of the function
fig, ax = plt.subplots()
ax.set_xlim(0, 96)  # Maze width is 96 inches
ax.set_ylim(0, 48)  # Maze height is 48 inches
ax.set_aspect('equal', 'box')
ax.set_xlabel('X (inches)')
ax.set_ylabel('Y (inches)')
plt.gca().invert_yaxis()  # Invert Y axis to match the coordinate system (Y increases downward)

# Draw the walls once (static content)
walls = [
    [48, 60, 0, 12], [72, 84, 0, 12], [24, 36, 12, 24],
    [12, 24, 24, 36], [36, 48, 24, 36], [48, 60, 24, 36],
    [72, 84, 24, 36], [72, 84, 36, 48]
]

for wall in walls:
    x_start, x_end, y_start, y_end = wall
    ax.add_patch(plt.Rectangle((x_start, y_start), x_end - x_start, y_end - y_start, color='gray'))

# Robot outline relative to its center
robot_outline = [
    pm.Vector2(-2.45, -2.45),
    pm.Vector2(-2.45,  2.45),
    pm.Vector2(-1.7,  3.12),
    pm.Vector2( 1.7,  3.12),
    pm.Vector2( 2.45,  2.45),
    pm.Vector2( 2.45, -2.45)
]

# Turn on interactive mode
plt.ion()

# Function to update the robot position and draw it
def draw_robot_with_maze(position, angle):
    """
    Draws the maze with walls (rectangles) and the robot at a given position and orientation.
    
    Args:
        position: A list [x, y] where the robot is located.
        angle: The robot's orientation in degrees (0 is facing downward, positive is clockwise).
    """
    # Convert angle to radians for rotation
    angle_rad = np.radians(angle)
    
    # Rotate and translate the robot outline
    rotated_outline = []
    for point in robot_outline:
        # Apply the rotation
        x_rot = point.x * np.cos(angle_rad) - point.y * np.sin(angle_rad)
        y_rot = point.x * np.sin(angle_rad) + point.y * np.cos(angle_rad)
        # Translate to the new position
        rotated_outline.append([x_rot + position[0], y_rot + position[1]])
    
    # Convert the rotated outline to a numpy array for plotting
    rotated_outline = np.array(rotated_outline)
    
    # Clear the previous robot outline if it exists
    for patch in getattr(draw_robot_with_maze, 'robot_patch', []):
        patch.set_visible(False)
    
    # Draw the robot outline on the plot
    draw_robot_with_maze.robot_patch = ax.fill(rotated_outline[:, 0], rotated_outline[:, 1], 'r', edgecolor='black', linewidth=2)  # Robot in red
    
    # Redraw the plot to update it
    plt.draw()
    plt.pause(3)  # Pause for a short moment to allow the plot to update
    
import heapq

# Grid and obstacles
grid_width, grid_height = 96, 48
buffer = 1  # Adjust this value to change the buffer size

prohibited_areas = [
    [max(0, 0 - buffer), min(96, 3 + buffer), max(0, 0 - buffer), min(48, 49 + buffer)],      # Left boundary wall
    [max(0, 93 - buffer), min(96, 96 + buffer), max(0, 0 - buffer), min(48, 49 + buffer)],    # Right boundary wall
    [max(0, 0 - buffer), min(96, 96 + buffer), max(0, 0 - buffer), min(48, 3 + buffer)],      # Top boundary wall
    [max(0, 0 - buffer), min(96, 96 + buffer), max(0, 46 - buffer), min(48, 49 + buffer)],    # Bottom boundary wall
    [max(0, 45 - buffer), min(96, 64 + buffer), max(0, 0 - buffer), min(48, 16 + buffer)],    # Inner wall 1
    [max(0, 69 - buffer), min(96, 89 + buffer), max(0, 0 - buffer), min(48, 16 + buffer)],    # Inner wall 2
    [max(0, 20 - buffer), min(96, 41 + buffer), max(0, 9 - buffer), min(48, 28 + buffer)],    # Inner wall 3
    [max(0, 8 - buffer), min(96, 29 + buffer), max(0, 21 - buffer), min(48, 40 + buffer)],    # Inner wall 4
    [max(0, 32 - buffer), min(96, 53 + buffer), max(0, 21 - buffer), min(48, 40 + buffer)],   # Inner wall 5
    [max(0, 44 - buffer), min(96, 64 + buffer), max(0, 21 - buffer), min(48, 40 + buffer)],   # Inner wall 6
    [max(0, 69 - buffer), min(96, 89 + buffer), max(0, 21 - buffer), min(48, 40 + buffer)],   # Inner wall 7
    [max(0, 69 - buffer), min(96, 89 + buffer), max(0, 33 - buffer), min(48, 52 + buffer)]    # Inner wall 8
]

# Convert prohibited areas to a set of obstacle cells for quick lookup
obstacles = set()
for area in prohibited_areas:
    x_start, x_end, y_start, y_end = area
    for x in range(x_start, x_end):
        for y in range(y_start, y_end):
            obstacles.add((x, y))

# A* pathfinding
def heuristic(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return dx + dy

def astar(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in [(0, -2), (2, 0), (0, 2), (-2, 0)]:  # Moves of 2 inches
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < grid_width and 0 <= neighbor[1] < grid_height 
                and neighbor not in obstacles):
                new_cost = cost_so_far[current] + 2
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

    return None  # No path found

# Convert the dictionary to the nested list format
Simulated_Lidar = [[[x, y, theta], lidar_readings] for (x, y, theta), lidar_readings in Mickel_Table.items()]
Probability_Matrix = [1/len(Simulated_Lidar) for _ in range(len(Simulated_Lidar))]
Sum_Matrix = [0 for _ in range(len(Simulated_Lidar))]

def Compare_Lidar(Current_Lidar):
    sigma = 2
    for i in range(len(Simulated_Lidar)):
        for n in range(36):
            #Sum_Matrix[i] += (Current_Lidar[n] - Simulated_Lidar[i][1][n])**2 / (2 * sigma**2)  
            Sum_Matrix[i] += abs(Current_Lidar[n] - Simulated_Lidar[i][1][n])           
    min_index = Sum_Matrix.index(min(Sum_Matrix))
    #Probability_Matrix[min_index] += 0.1
    #max_index = Probability_Matrix.index(max(Probability_Matrix))
    return Simulated_Lidar[min_index][0]

def Update_Positions(command):
    cmd_type, value = command.split(':')
    for i in range(len(Simulated_Lidar)):
        if cmd_type == 'w0':
            Simulated_Lidar[i][0][0] += int(value) * math.sin(math.radians(360 - Simulated_Lidar[i][0][2]))
            Simulated_Lidar[i][0][1] += int(value) * math.cos(math.radians(360 - Simulated_Lidar[i][0][2]))
        elif cmd_type == 'r0':
            Simulated_Lidar[i][0][2] = (Simulated_Lidar[i][0][2] + int(value)) % 360
    return None

drop_off_zones = {
    1: (66, 6),
    2: (90, 6),
    3: (30, 30),
    4: (90, 42)
}

print("Select a drop-off zone by entering a number (1-4):")
print("1: (66, 6)")
print("2: (90, 6)")
print("3: (30, 30)")
print("4: (90, 42)")
choice = int(input("Enter your choice (1-4): "))
selected_drop_off = drop_off_zones[choice]

# Generate robot commands considering orientation
def generate_robot_commands(path, initial_orientation):
    if path is None:
        print("Error: The provided path is None.")
        return []

    directions = {
        (0, -2): 180,  # Up
        (2, 0): 270,   # Right
        (0, 2): 0,     # Down
        (-2, 0): 90    # Left
    }
    current_orientation = initial_orientation
    commands = []
    for i in range(1, len(path)):
        current_pos = path[i - 1]
        next_pos = path[i]
        dx = next_pos[0] - current_pos[0]
        dy = next_pos[1] - current_pos[1]
        target_orientation = directions[(dx, dy)]

        # Calculate the rotation needed
        rotation_needed = (target_orientation - current_orientation) % 360
        if rotation_needed > 180:
            rotation_needed -= 360

        # Add rotation command if needed
        if rotation_needed != 0:
            commands.append(f"r0:{rotation_needed}")
            current_orientation = target_orientation

        # Add forward movement command
        commands.append(f"w0:2")

    return commands

# Get path and commands
def get_path(initial, goal):
    path = astar((initial[0], initial[1]), goal)
    if not path:
        print("No path found")
        return []
    commands = generate_robot_commands(path, initial[2])
    return commands

# Example usage
# start_position = (90, 42, 0)  # Starting position with orientation
# loading_zone = (6, 42)        # Target position
# commands_to_lz = get_path(start_position, loading_zone)
# print(commands_to_lz)


# Replace 'COM5' with your Bluetooth port
# bluetooth = serial.Serial("COM6", 9600, timeout=1)
bluetooth = serial.Serial("COM6",9600,timeout=5)

def send_command(command, max_retries=5):
    try:
        # Send command to Arduino
        bluetooth.write((command + "\n").encode())
        print(f"Sent command: {command}")

        for attempt in range(max_retries):
            # Wait for acknowledgment
            ack = bluetooth.readline().decode().strip()
            if ack == "ACK":
                print("Command acknowledged.")
                break
            else:
                print(f"No acknowledgment. Retrying ({attempt + 1}/{max_retries})...")
                bluetooth.write((command + "\n").encode())
                time.sleep(0.3)
        else:
            print(f"Failed to get acknowledgment for {command}.")
            return None

        # Handle sensor-specific commands
        if command in ["[LD]", "[UD]"]:
            data = bluetooth.readline().decode().strip()
            if data:
                if command == "[LD]":
                    distances = [float(d) if d != '-1' else 5 for d in data.split(",")]
                    print("LIDAR Distances:", distances)
                    return distances
                elif command == "[UD]":
                    values = [float(val) for val in data.split(",")]
                    print("Ultrasonic Distances:", values)
                    return values
            else:
                print(f"No data received for {command}. Retrying...")
        return None

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except ValueError as e:
        print(f"Data parsing error: {e}")
    return None

RUN_MANUAL = True
while RUN_MANUAL:
    cmd = input("Type in command (L, U, W, D, P, or a number): ").strip().upper()
    if cmd == "L":
        Lidar_Data = send_command("[LD]")
        if Lidar_Data:
            Lidar_Data = Lidar_Data[::-1]
            lidar_angles = list(range(0, 360, 10))
            actual_lidar = dict(zip(lidar_angles, Lidar_Data[:36]))
            print("LiDAR Data:", actual_lidar)
            try:
                Estimated_Position = Compare_Lidar(Lidar_Data)
                print(f"Estimated Position: x = {Estimated_Position[0]} inches, y = {Estimated_Position[1]} inches, theta = {Estimated_Position[2]} degrees")
                draw_robot_with_maze([Estimated_Position[0], Estimated_Position[1]], Estimated_Position[2])
            except Exception as e:
                print(f"Error processing LiDAR data: {e}")
        else:
            print("Failed to retrieve valid LiDAR data.")

    elif cmd == "U":
        UT_Data = send_command("[UD]")
        if UT_Data:
            print("Ultrasonic Data:", UT_Data)
        else:
            print("Failed to retrieve ultrasonic data.")

    elif cmd == "W":
        send_command("w0:5")

    elif cmd == "D":
        send_command("bp:0")

    elif cmd == "P":
        send_command("bp:1")

    else:
        try:
            float_cmd = float(cmd)  # Validate input as a number
            send_command("r0:" + str(float_cmd))
        except ValueError:
            print("Invalid command. Please try again.")

