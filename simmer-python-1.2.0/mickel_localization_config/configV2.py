# '''
# This file stores the configuration information for the simulator.
# '''
# # This file is part of SimMeR, an educational mechatronics robotics simulator.
# # Initial development funded by the University of Toronto MIE Department.
# # Copyright (C) 2023  Ian G. Bennett
# #
# # This program is free software: you can redistribute it and/or modify
# # it under the terms of the GNU Affero General Public License as published
# # by the Free Software Foundation, either version 3 of the License, or
# # (at your option) any later version.
# #
# # This program is distributed in the hope that it will be useful,
# # but WITHOUT ANY WARRANTY; without even the implied warranty of
# # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# # GNU Affero General Public License for more details.
# #
# # You should have received a copy of the GNU Affero General Public License
# # along with this program.  If not, see <https://www.gnu.org/licenses/>.

import pygame.math as pm
from devices.motors import MotorSimple
from devices.ultrasonic import Ultrasonic

from devices.gyroscope import Gyroscope
from devices.compass import Compass
from devices.infrared import Infrared
from devices.drive import Drive

# Control Flags and Setup
num_robots =1000 # Set the default number of robots
# Robot initial positioning range
robot_initial_position_range = [0, 50]  # Adjust for maze dimensions
robot_initial_rotation_range = [0, 360]  # Allow any rotation from 0 to 360 degrees

# Control Flags and Setup
rand_error = False          # Use either true random error generator (True) or repeatable error generation (False)
rand_bias = False           # Use a randomized, normally distributed set of bias values for drives (placeholder, not implemented)
bias_strength = [0.05, 1]   # How intense the random drive bias is, if enabled (placeholder, not implemented)

# Network configuration for sockets
host = '127.0.0.1'
port_rx = 61200
port_tx = 61201
timeout = 300
str_encoding = 'ascii'
frame_start = '['
frame_end = ']'

# General communication settings
round_digits = 3

# Block information
block_position = [66, 5]        # Block starting location
block_rotation = 0              # Block rotation (deg)
block_size = 3                  # Block side length in inches

# Robot information
robot_start_position = [6, 42] # Robot starting location (in)
robot_start_rotation = 180      # Robot starting rotation (deg)
robot_width = 3           # Robot width in inches
robot_height = 7.55              # Robot height in inches
robot_outline = [               # Robot outline, relative to center position
                pm.Vector2(-3, -3),
                pm.Vector2(-3,  3),
                pm.Vector2( -1.7,  4.2),
                pm.Vector2( 1.7, 4.2),
                pm.Vector2( 3,  3),
                pm.Vector2( 3, -3)
                ]

# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[3,3,1,1,0,2,0,2],
         [3,3,0,1,1,1,1,1],
         [1,0,2,0,0,1,0,1],
         [1,1,1,1,1,1,0,2]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correctfloor pattern
maze_dim_x = len(walls[0])*wall_segment_length
maze_dim_y = len(walls)*wall_segment_length


# Graphics information
frame_rate = 60             # Target frame rate (Hz)
ppi = 16                  # Number of on-screen pixels per inch on display
border_pixels = floor_segment_length * ppi  # Size of the border surrounding the maze area

background_color = (43, 122, 120)

wall_thickness = 0.25       # Thickness to draw wall segments, in inches
wall_color = (255, 0, 0)    # Tuple with wall color in (R,G,B) format

robot_thickness = 0.25      # Thickness to draw robot perimeter, in inches
robot_color = (0, 0, 255)   # Tuple with robot perimeter color in (R,G,B) format

block_thickness = 0.25      # Thickness to draw robot perimeter, in inches
block_color = (127, 127, 0) # Tuple with robot perimeter color in (R,G,B) format



### DEVICE CONFIGURATION ###
# Motors
m0_info = {
    'id': 'm0',
    'position': [2.45, -2.45],
    'rotation': 0,
    'visible': True,
    'color': (128, 128, 0)
}

m1_info = {
    'id': 'm0',
    'position': [-2.45, -2.45],
    'rotation': 0,
    'visible': True,
    'color': (0, 128, 0)
}

m2_info = {
    'id': 'm0',
    'position': [0, 2],
    'rotation': 90,
    'visible': False,
    'color': (128, 0, 0)
}

m3_info = {
    'id': 'm0',
    'position': [0, -2],
    'rotation': 90,
    'visible': False,
    'color': (0, 0, 128)
}

motors = {
    'm0': MotorSimple(m0_info),
    'm1': MotorSimple(m1_info),
    'm2': MotorSimple(m2_info),
    'm3': MotorSimple(m3_info)
}
# Drives
w0_info = {
    'id': 'w0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 20],
    'ang_velocity': 0,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, 1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0.2},
    'error': {'x': 0.02, 'y': 0.05, 'rotation': 1}
}

d0_info = {
    'id': 'd0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [-12, 0],
    'ang_velocity': 0,
    'motors': [motors['m2'], motors['m3']],
    'motor_direction': [1, 1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0},
    'error': {'x': 0, 'y': 0, 'rotation': 0}
}

r0_info = {
    'id': 'r0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 0],
    'ang_velocity': 30,
    'motors': [motors['m0'], motors['m1'], motors['m2'], motors['m3']],
    'motor_direction': [1, -1, 1, -1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0.01},
    'error': {'x': 0.003, 'y': 0.003, 'rotation': 0.02}
}

drives = {
    'w0': Drive(w0_info),
    'd0': Drive(d0_info),
    'r0': Drive(r0_info)
}
import pygame.math as pm
import math

def create_sensors(num_measurements):
    sensors = {}
    radius = 2.5  # Distance from the robot's center to the sensor
    error = 0  # Error value for all sensors
    height = 1    # Height of sensors

    for i in range(num_measurements):
        angle = ((360 / num_measurements) * i)  # Calculate angle for current sensor
        radians = math.radians(angle)  # Convert angle to radians
        #position = [radius * math.sin(radians), radius * math.cos(radians)]  # Calculate position
        position = [0,0]
        sensor_id = str(angle) + ' deg'
        #sensor_id = f'u{i}'  # Unique ID for each sensor
        sensors[sensor_id] = {
            'id': sensor_id,
            'position': position,
            'height': height,
            'rotation': angle,
            'error': error,
            'outline': [
                pm.Vector2(-1, -0.5),
                pm.Vector2(-1, 0.5),
                pm.Vector2(1, 0.5),
                pm.Vector2(1, -0.5)
            ],
            'visible': True,
            'visible_measurement': True
        }
    # u0_info  = {
    #         'id': 'u0',
    #         'position': [0,4.17],
    #         'height': 1,
    #         'rotation': 0,
    #         'error': 0,
    #         'outline': [
    #             pm.Vector2(-1, -0.5),
    #             pm.Vector2(-1, 0.5),
    #             pm.Vector2(1, 0.5),
    #             pm.Vector2(1, -0.5)
    #         ],
    #         'visible': True,
    #         'visible_measurement': True
    # }
    # u1_info  = {
    #         'id': 'u0',
    #         'position': [-2.9,1],
    #         'height': 1,
    #         'rotation': 90,
    #         'error': 0,
    #         'outline': [
    #             pm.Vector2(-1, -0.5),
    #             pm.Vector2(-1, 0.5),
    #             pm.Vector2(1, 0.5),
    #             pm.Vector2(1, -0.5)
    #         ],
    #         'visible': True,
    #         'visible_measurement': True
    # }
    # u2_info  = {
    #         'id': 'u0',
    #         'position': [2.9,1],
    #         'height': 1,
    #         'rotation': -90,
    #         'error': 0,
    #         'outline': [
    #             pm.Vector2(-1, -0.5),
    #             pm.Vector2(-1, 0.5),
    #             pm.Vector2(1, 0.5),
    #             pm.Vector2(1, -0.5)
    #         ],
    #         'visible': True,
    #         'visible_measurement': True
    # }
    # u3_info  = {
    #         'id': 'u0',
    #         'position': [-2.3, 2.9],
    #         'height': 1,
    #         'rotation': -45,
    #         'error': 0,
    #         'outline': [
    #             pm.Vector2(-1, -0.5),
    #             pm.Vector2(-1, 0.5),
    #             pm.Vector2(1, 0.5),
    #             pm.Vector2(1, -0.5)
    #         ],
    #         'visible': True,
    #         'visible_measurement': True
    # }
    # u4_info  = {
    #         'id': 'u0',
    #         'position': [2.3,2.9],
    #         'height': 1,
    #         'rotation': 45,
    #         'error': 0,
    #         'outline': [
    #             pm.Vector2(-1, -0.5),
    #             pm.Vector2(-1, 0.5),
    #             pm.Vector2(1, 0.5),
    #             pm.Vector2(1, -0.5)
    #         ],
    #         'visible': True,
    #         'visible_measurement': True
    # }
    # sensors['u0'] = u0_info
    # sensors['u1'] = u1_info
    # sensors['u2'] = u2_info
    # sensors['u3'] = u3_info
    # sensors['u4'] = u4_info
    return sensors
# Example usage
num_measurements = 36  # Specify the number of sensors
sensor_data = create_sensors(num_measurements)



# g0_info = {
#     'id': 'u0',
#     'position': [0, 0],
#     'rotation': 0,
#     'error': 0.02,
#     'bias': 0.1,
#     'visible': False
# }

# c0_info = {
#     'id': 'c0',
#     'position': [0, 0],
#     'rotation': 0,
#     'error': 0.02,
#     'bias': 0.1,
#     'visible': False
# }

# i0_info = {
#     'id': 'i0',
#     'position': [0, -1],
#     'height': 1.5,
#     'rotation': 0,
#     'fov': 60,
#     'threshold': 0.7,
#     'error': 0.05,
#     'bias': 0.1,
#     'color': (127, 127, 127),
#     'visible': False,
#     'visible_measurement': False
#}
sensors = {sensor_id: Ultrasonic(info) for sensor_id, info in sensor_data.items()}
# ### TESTING AND DEBUG SETTINGS ###
# ###simulate_list = ['u0', 'u1','u2','u3', 'i0']
######################################################################################################################

