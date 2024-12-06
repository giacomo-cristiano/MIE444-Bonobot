
# Load your precomputed matrix with positions, orientations, and LiDAR readings
# Format for each entry in map_lidar_data: [x, y, theta, lidar_readings]
# Original data format as a single dictionary

import socket
import time
from datetime import datetime
import serial
# import ast
# import random
import numpy as np
import math
import matplotlib.pyplot as plt
import pygame.math as pm  # Assuming you have the 'pygame' library for the Vector2 class

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

# Replace 'COM5' with your Bluetooth port
# bluetooth = serial.Serial("COM6", 9600, timeout=1)
bluetooth = serial.Serial("COM6",9600,timeout=5)
# def send_command(command):
#     # Send the command over Bluetooth
#     bluetooth.write((command + "\n").encode())
#     print(f"Sent command: {command}")

#     # Unified retry logic
#     retries = 5  # Maximum retries for acknowledgment or data reception
#     output = None

#     for attempt in range(retries):
#         # Check for acknowledgment first
#         ack = bluetooth.readline().decode().strip()
#         if ack == "ACK":
#             print("Command acknowledged by Arduino.")
#         else:
#             # Skip retries for specific drive commands (e.g., w0, r0, xx)
#             if command.startswith("w0") or command.startswith("r0") or command.startswith("xx"):
#                 print(f"No acknowledgment for drive command: {command}. Skipping retries.")
#                 return None
#             print(f"No acknowledgment received. Retrying... ({attempt + 1}/{retries})")
#             bluetooth.write((command + "\n").encode())
#             time.sleep(0.3)
#             continue

#         # Handle specific responses for sensor data
#         if command in ["LD", "UD"]:  # No brackets required for commands
#             data = bluetooth.readline().decode().strip()
#             if data:
#                 if command == "LD":
#                     distances = data.split(",")
#                     distances = [float(d) if d != '-1' else 5 for d in distances]
#                     print("LIDAR Distances:", distances)
#                     output = distances
#                 elif command == "UD":
#                     sensor_values = data.split(",")
#                     sensor_values = [float(val) for val in sensor_values]
#                     print("Ultrasonic Sensor Distances (inches):", sensor_values)
#                     output = sensor_values
#                 break  # Exit loop after processing data
#             else:
#                 print(f"No data received for {command}. Retrying... ({attempt + 1}/{retries})")
#                 bluetooth.write((command + "\n").encode())
#                 time.sleep(0.2)
#                 continue
#         else:
#             # No additional response processing for non-sensor commands
#             break

#     if output is None and command in ["LD", "UD"]:
#         print(f"Failed to receive valid data for {command} after {retries} attempts.")
#     elif output is None:
#         print("Command executed but no data expected or returned.")

#     return output

def send_command(command):
    bluetooth.write((command + "\n").encode())
    print(f"Sent command: {command}")

    # Unified retry logic for both acknowledgment and data
    retries = 5  # Maximum retries for acknowledgment or data reception
    output = None

    for attempt in range(retries):
        # Check for acknowledgment first
        ack = bluetooth.readline().decode().strip()
        if ack == "ACK":
            print("Command acknowledged by Arduino.")
        elif "w0" in command or "r0" in command or "xx" in command:
            # Skip retries for drive commands if no acknowledgment is received
            print(f"No acknowledgment for drive command: {command}. Skipping retries.")
            return None
        else:
            print(f"No acknowledgment received. Retrying... ({attempt + 1}/{retries})")
            bluetooth.write((command + "\n").encode())
            time.sleep(0.3)
            continue

        # Handle specific responses for sensor data
        if command in ["[LD]", "[UD]"]:
            data = bluetooth.readline().decode().strip()
            if command == "[LD]":
                time.sleep(0.5)
            if command == "[UD]":
                time.sleep(0.2)
            if data:
                if command == "[LD]":
                    distances = data.split(",")
                    distances = [float(d) if d != '-1' else 5 for d in distances]
                    print("LIDAR Distances:", distances)
                    output = distances
                elif command == "[UD]":
                    sensor_values = data.split(",")
                    sensor_values = [float(val) for val in sensor_values]
                    print("Ultrasonic Sensor Distances (inches):", sensor_values)
                    output = sensor_values
                break  # Exit the loop after successfully processing data
            else:
                print(f"No data received for {command}. Retrying... ({attempt + 1}/{retries})")
                bluetooth.write((command + "\n").encode())
                time.sleep(0.2)
                continue
        else:
            # No additional response processing for non-sensor commands
            break

    if output is None and command in ["[LD]", "[UD]"]:
        print(f"Failed to receive valid data for {command} after {retries} attempts.")
    elif output is None:
        print("Command executed but no data expected or returned.")

    return output

# Constants
THRESHOLD_DISTANCE = 2.5
THRESHOLD_DISTANCE1 = 3.5
THRESHOLD_DISTANCE2 = 1
MAX_SAME_MOVES = 3       # Threshold for repeating the same move

# Initialize movement history
movement_history = []

def decide_next_move(sensor_data):
    front, right, left, back, fr, fl = sensor_data[:6]
    
    # Detect if there are obstacles
    obstacles = {
        'front': front < THRESHOLD_DISTANCE,
        'left': left < THRESHOLD_DISTANCE2,
        'right': right < THRESHOLD_DISTANCE2,
        'fl': fl < THRESHOLD_DISTANCE1,
        'fr': fr < THRESHOLD_DISTANCE1,
        'back': back < THRESHOLD_DISTANCE,
    }

    # Last move
    last_move = movement_history[-1] if movement_history else None

    # Decision logic
    if obstacles['front'] or obstacles['fl'] or obstacles['fr'] or obstacles['left'] or obstacles['right']:
        if last_move == 'turn left':
            next_move = 'turn left'
        elif last_move == 'turn right':
            next_move = 'turn right'
        else:
            if (right+fr)/2 > (left+fl)/2:
                 next_move = 'turn right'
            else:
                next_move = 'turn left'
    else:
        next_move = 'proceed path'  # No obstacles detected

    # Update movement history
    movement_history.append(next_move)
    if len(movement_history) > MAX_SAME_MOVES:
        movement_history.pop(0)
    return next_move

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

        for dx, dy in [(0, -3), (3, 0), (0, 3), (-3, 0)]:  # Moves of 3 inches
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < grid_width and 0 <= neighbor[1] < grid_height 
                and neighbor not in obstacles):
                new_cost = cost_so_far[current] + 3
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

    return None  # No path found

# Generate robot commands considering orientation
def generate_robot_commands(path, initial_orientation):
    if path is None:
        print("Error: The provided path is None.")
        return []

    directions = {
        (0, -3): 180,  # Up
        (3, 0): 270,   # Right
        (0, 3): 0,     # Down
        (-3, 0): 90    # Left
    }
    current_orientation = initial_orientation
    commands = []
    pending_move = 0

    for i in range(1, len(path)):
        current_pos = path[i - 1]
        next_pos = path[i]
        dx = next_pos[0] - current_pos[0]
        dy = next_pos[1] - current_pos[1]
        target_orientation = directions[(dx, dy)]

        # Calculate the rotation needed and ensure it's a multiple of 10 degrees
        rotation_needed = (target_orientation - current_orientation) % 360
        if rotation_needed > 180:
            rotation_needed -= 360
        rounded_rotation = (round(rotation_needed / 10) * 10) if abs(rotation_needed) >= 10 else 0

        # Execute the rotation if needed
        if rounded_rotation != 0:
            if pending_move > 0:
                commands.append(f"w0:{pending_move}")
                pending_move = 0
            commands.append(f"r0:{rounded_rotation}")
            current_orientation = (current_orientation + rounded_rotation) % 360

        # Add the forward movement (always in multiples of 3 inches)
        move_distance = abs(dx) + abs(dy)
        pending_move += move_distance

    # Handle any remaining movement
    if pending_move > 0:
        commands.append(f"w0:{pending_move}")

    return commands

# Combine forward movements to ensure they meet minimum step size
def combine_forward_movements(commands):
    combined_commands = []
    pending_distance = 0

    for command in commands:
        if command.startswith("w0:"):
            distance = int(command.split(":")[1])
            pending_distance += distance
        else:
            if pending_distance >= 3:
                combined_commands.append(f"w0:{pending_distance}")
                pending_distance = 0
            combined_commands.append(command)

    # Handle any remaining distance
    if pending_distance >= 3:
        combined_commands.append(f"w0:{pending_distance}")

    return combined_commands

# Get path and commands
def get_path(initial, goal):
    path = astar((initial[0], initial[1]), goal)
    if not path:
        print("No path found")
        return []
    commands = generate_robot_commands(path, initial[2])
    return combine_forward_movements(commands)

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

RUN_MANUAL = True
while RUN_MANUAL:
    
# GATHER LIDAR DATA: 
    Lidar_Data = send_command("[LD]")
    Lidar_Data = Lidar_Data[::-1]
    # Extract only the LiDAR readings and convert to dictionary format
    lidar_angles = list(range(0, 360, 10))
    actual_lidar = dict(zip(lidar_angles, Lidar_Data[:36]))
    print("LiDAR Data:", actual_lidar)
    Estimated_Position = Compare_Lidar(Lidar_Data)
    Sum_Matrix = [0 for _ in range(len(Simulated_Lidar))]
    print(f"Estimated Position: x = {Estimated_Position[0]} inches, y = {Estimated_Position[1]} inches, theta = {Estimated_Position[2]} degrees")
    start_position = (Estimated_Position[0], Estimated_Position[1], Estimated_Position[2])
    loading_zone = (12,12)
    commands_to_lz  = get_path(start_position, loading_zone)
    Path = combine_forward_movements(commands_to_lz)
    print("The Path is: " + str(Path))
    draw_robot_with_maze([Estimated_Position[0], Estimated_Position[1]], Estimated_Position[2])
    
# PROVIDE MANUAL INPUT
    cmd = input("Type in drive command: ")
    drive_cmd = "[" + cmd + "]"
    send_command(drive_cmd)
    OBSTACLE_AVOID = False
    # Define the maximum time in seconds
    max_time = 5  # Example: exit the loop after 10 seconds
    # Record the start time
    start_time = time.time()
    while OBSTACLE_AVOID:
        elapsed_time = time.time() - start_time
        if elapsed_time >= max_time:
            print(f"Exiting loop after {elapsed_time:.2f} seconds.")
            break
        UT_Data = send_command("[UD]")
        #time.sleep(0.3)
        next_move = decide_next_move(UT_Data)
        print(next_move)
        if next_move == 'move backward':
            send_command("[xx]")
            time.sleep(3)
            #send_command("[w0:-2]")
            OBSTACLE_AVOID = False
        elif next_move == 'turn left':
            send_command("[xx]")
            time.sleep(3)
            #send_command("[r0:-15]")
            OBSTACLE_AVOID = False
        elif next_move == 'turn right':
            send_command("[xx]")
            time.sleep(3)
            #send_command("[r0:15]")
            OBSTACLE_AVOID = False


############## Main section for Autonomous Control ##############
RUN_AUTONOMOUS = False # If true, run this. If false, skip it
Stage = 0       #   0 - navigate to loading zone, 1 - block detection, 2 - navigate to drop-off zone
OBSTACLE_AVOID = False
while RUN_AUTONOMOUS:
    Lidar_Data = send_command("[LD]")
    Lidar_Data = Lidar_Data[::-1]
    # Extract only the LiDAR readings and convert to dictionary format
    lidar_angles = list(range(0, 360, 10))
    actual_lidar = dict(zip(lidar_angles, Lidar_Data[:36]))
    print("LiDAR Data:", actual_lidar)
    Estimated_Position = Compare_Lidar(Lidar_Data)
    Sum_Matrix = [0 for _ in range(len(Simulated_Lidar))]
    print(f"Estimated Position: x = {Estimated_Position[0]} inches, y = {Estimated_Position[1]} inches, theta = {Estimated_Position[2]} degrees")
    start_position = (Estimated_Position[0], Estimated_Position[1], Estimated_Position[2])
    
    if Stage == 0:
        loading_zone = (12, 12)
        target = loading_zone
        commands_to_lz  = get_path(start_position, target)
        
    elif Stage == 1:
        # Block Detection and Pick-Up
        print("Block Detected")
        Stage += 1
        continue        # Skip current iteration and re-localize
    
    elif Stage == 2:
        target = selected_drop_off
        commands_to_lz  = get_path(start_position, target)
        
    else:
        print("Block Droped-Off")
        break
        
    Path = combine_forward_movements(commands_to_lz)
    if len(Path) == 1:
        Stage +=1
        
    draw_robot_with_maze([Estimated_Position[0], Estimated_Position[1]], Estimated_Position[2])  # Example position and angle (180 degrees)
    send_command("[" + Path[0] + "]")
    OBSTACLE_AVOID = True
    # Define the maximum time in seconds
    max_time = 5  # Example: exit the loop after 10 seconds
    # Record the start time
    start_time = time.time()
    while OBSTACLE_AVOID:
        elapsed_time = time.time() - start_time
        if elapsed_time >= max_time:
            print(f"Exiting loop after {elapsed_time:.2f} seconds.")
            break
        UT_Data = send_command("[UD]")
        time.sleep(0.3)
        next_move = decide_next_move(UT_Data)
        print(next_move)
        if next_move == 'move backward':
            send_command("[xx]")
            time.sleep(3)
            send_command("[w0:-2]")
            OBSTACLE_AVOID = False
        elif next_move == 'turn left':
            send_command("[xx]")
            time.sleep(3)
            send_command("[r0:-15]")
            OBSTACLE_AVOID = False
        elif next_move == 'turn right':
            send_command("[xx]")
            time.sleep(3)
            send_command("[r0:15]")
            OBSTACLE_AVOID = False