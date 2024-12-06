'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import socket
import time
from datetime import datetime
import serial

# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response {item[1][1]} \n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid


############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0

############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = False # If true, run this. If false, skip it
while RUN_COMMUNICATION_CLIENT:
    # Input a command
    cmd = input('Type in a string to send: ')

    # Send the command
    packet_tx = packetize(cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive the response
    [responses, time_rx] = receive()
    if responses[0]:
        print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
    else:
        print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")

############## Main section for the open loop control algorithm ##############

LOOP_PAUSE_TIME = 0.2  # seconds
STOPPING_DISTANCE = 3  # inches for obstacle avoidance 
MOVEMENT_INCREMENT = 2  # Inch increment for moving forward
TARGET_LEFT_DISTANCE = 2.5  # Target distance in inches to maintain from the left wall (u2 sensor)
KP = 0.5  # Proportional gain for lateral movement correction

# Function to check all ultrasonic sensors
def check_all_sensors():
    sensors = ['u0', 'u1', 'u2', 'u3', 'u4', 'u5']
    readings = {}
    obstacle_detected = False  # Flag to track if any obstacle is within STOPPING_DISTANCE

    for sensor in sensors:
        packet_tx = packetize(sensor)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            
            if responses[0] and isinstance(responses[0][1], str):
                try:
                    ultrasonic_reading = float(responses[0][1])
                    readings[sensor] = ultrasonic_reading
                    print(f"{sensor} reading: {ultrasonic_reading} inches")
                    
                    # Check if the reading is below the stopping distance threshold
                    if ultrasonic_reading <= STOPPING_DISTANCE:
                        print(f"Obstacle detected by {sensor} within {STOPPING_DISTANCE} inches!")
                        obstacle_detected = True  # Set flag to True if any obstacle is detected
                except ValueError:
                    print(f"Invalid sensor reading from {sensor}: {responses[0][1]}")
            else:
                print(f"Malformed packet or no valid data from {sensor}.")

    return readings, not obstacle_detected  # Return all readings and whether it's safe to move


# Function to perform obstacle avoidance
def obstacle_avoidance(sensor_readings):

    # Algorithm is dominant for hugging the left side of the maze. 
    #TURNING CONDITIONS
    
    ## CW TURN
    if u2_reading < 6 and (u0_reading < STOPPING_DISTANCE):  # left wall + Obstacle ahead 
            print("Turning CW.")
            return packetize('r0:85')  # Move right    
    print("No obstacle in front")
    
    ## CCW TURN 
    if u2_reading > 6 and (u0_reading < STOPPING_DISTANCE):  # no left wall + Obstacle ahead
            print("Turning CCW.")
            return packetize('r0:-100')  # Move right    
    print("No obstacle in front")
    
    # Side Wall Avoidance during translation
    if (u1_reading < STOPPING_DISTANCE or u4_reading < STOPPING_DISTANCE):  # Obstacle on the right
        print("Obstacle on the right! Shifting left.")
        return packetize('d0:-1.5')  # Move left
    if u5_reading < STOPPING_DISTANCE:
        print("Obstacle on the left ! Shifting right.")
        return packetize('d0:1.5')  # Move left
    
# Function to regulate distance from left wall
def regulate_left_distance(sensor_readings2):
    # target left distance is our desired setpoint 
    # u2_reading = sensor_readings2.get('u2', float('inf'))  # West (left)
    if sensor_readings2.get('u2', float('inf')) <= 6:
        error = u2_reading - TARGET_LEFT_DISTANCE  # Error: how far we are from the target distance
        lateral_movement = KP * error  # Proportional control action, KP determines the sensitivity of corrections
    # Only apply correction if enough time has passed since the last correction

                # Send lateral correction commands based on the error
        if abs(error) > 0.2:  # Threshold to avoid minor corrections
            if lateral_movement > 0:
                print(f"Too far from left wall! Moving left by {lateral_movement} inches.")
                        
                return packetize(f'd0:{-min(lateral_movement, 0.5)}') # Move left, capped to 1.5 inches
            elif lateral_movement < 0:
                print(f"Too close to left wall! Moving right by {-lateral_movement} inches.")
                    
                return packetize(f'd0:{min(-lateral_movement, 0.5)}')   # Move right, capped to 1.5 inches
        return None  # No correction needed

# Main loop for wall-following with left distance regulation and turn handling
RUN_DEAD_RECKONING = True

while RUN_DEAD_RECKONING:
    # Check sensor readings before making any movement decision
    sensor_readings, safe_to_move = check_all_sensors()
    
    u0_reading = sensor_readings.get('u0', float('inf'))  # North (front)
    u1_reading = sensor_readings.get('u1', float('inf'))  # East (right)
    u2_reading = sensor_readings.get('u2', float('inf'))  # West (left)
    u3_reading = sensor_readings.get('u3', float('inf'))  # South (back)
    u4_reading = sensor_readings.get('u4', float('inf'))  # Front-right
    u5_reading = sensor_readings.get('u5', float('inf'))  # Front-left

    # Prioritize obstacle avoidance and turning first
    if not safe_to_move:
        avoidance_packet = obstacle_avoidance(sensor_readings)
        if avoidance_packet:
            transmit(avoidance_packet)
            time.sleep(LOOP_PAUSE_TIME)
            [responses, time_rx] = receive()
            print(f"Avoidance command response: {response_string(avoidance_packet, responses)}")
            # continue  # Skip to the next loop iteration after avoidance
            
    # Regulate distance from the left wall (using u2 sensor)
    regulation_packet = regulate_left_distance(sensor_readings)
    # regulation_packet = regulate_left_distance(sensor_readings)
    if regulation_packet:
        transmit(regulation_packet)
        time.sleep(LOOP_PAUSE_TIME)
        [responses, time_rx] = receive()
        print(f"Regulation command response: {response_string(regulation_packet, responses)}")
        # continue  # Recheck sensors after lateral correction

    # Safe to move forward, continue with movement
    remaining_distance = MOVEMENT_INCREMENT
    while remaining_distance > 0:
        # Move robot forward by the increment
        incremental_command = f'w0:{min(MOVEMENT_INCREMENT, remaining_distance)}'
        packet_tx = packetize(incremental_command)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(LOOP_PAUSE_TIME)
            [responses, time_rx] = receive()
            print(f"Drive command response: {response_string(incremental_command, responses)}")

        # Decrease remaining distance by the increment
        remaining_distance -= MOVEMENT_INCREMENT

    # Simulate a pause between movements
    time.sleep(LOOP_PAUSE_TIME)

# When the loop completes, end the program
RUN_DEAD_RECKONING = False
print("Navigation complete!")
