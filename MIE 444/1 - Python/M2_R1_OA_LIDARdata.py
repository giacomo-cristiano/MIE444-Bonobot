import socket
import time
from datetime import datetime
import serial
import ast

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
                
            # Print debug messages directly
            if response_char.startswith('Debug: '):  # Check for debug messages
                print(response_char.strip())  # Print debug messages

            if response_char == FRAMEEND:
                break

    print(f'Raw response was: {response_raw}')

    if response_raw:
        #return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
        return [response_raw, datetime.now().strftime("%H:%M:%S")]
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
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    if (start >= 0 and end >= start):
        data = data_raw[start + 1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

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
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

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

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

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
PORT_SERIAL = 'COM5'    # COM port identification
TIMEOUT_SERIAL = 0.75    # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = False

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
RUN_COMMUNICATION_CLIENT = True  # If true, run this. If false, skip it
loop_start_time = time.perf_counter
while RUN_COMMUNICATION_CLIENT:
    
    iteration_start = time.perf_counter()  # Get the current time with high precision
    
    packet_tx = packetize("LD")
    transmit(packet_tx)
    time.sleep(10)  # Wait for the response
    [responses, time_rx] = receive()

    print("Bonobo test response is: " + str(responses))
    
    iteration_end = time.perf_counter()
    
    # Calculate the time difference
    time_difference = iteration_end - iteration_start
    print(f"Time between iterations: {time_difference:.6f} seconds")
    
    #Input a command
    cmd = input('Type in a string to send: ')

    # Send the command
    packet_tx = packetize(cmd)
    if packet_tx:
        print(f"Transmitting command: {packet_tx}")  # Debug line
        transmit(packet_tx)
        time.sleep(10)
        
import random
# Constants
THRESHOLD_DISTANCE = 1.5 # front 
THRESHOLD_DISTANCE1 = 3.5 # diagonal 
THRESHOLD_DISTANCE2 = 1 # lateral 
MAX_SAME_MOVES = 3       # Threshold for repeating the same move

# Initialize movement history
movement_history = []

def decide_next_move(sensor_data):
    front, right, left, back, fr, fl = sensor_data
    
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
                
        # If front or angled sensors detect obstacles
        if not obstacles['left'] and last_move != 'left':
            next_move = 'turn left'
        elif not obstacles['right'] and last_move != 'right':
            next_move = 'turn right'
        elif not obstacles['fl'] and last_move != 'move forward':
            next_move = 'move forward'  # If forward left is clear
        elif not obstacles['fr'] and last_move != 'move forward':
            next_move = 'move forward'  # If forward right is clear
        else:
            # If all directions are blocked, back up or turn randomly
            if last_move != 'move backward':
                next_move = 'move backward'
            else:
                if right>left:
                    next_move = 'turn right'
                else:
                    next_move = 'turn left'
                #next_move = random.choice(['turn left', 'turn right'])
    else:
        if left > front and left > right:
            next_move = 'rotate left'
        else:
            next_move = 'move forward'  # No obstacles detected

    # Update movement history
    movement_history.append(next_move)
    if len(movement_history) > MAX_SAME_MOVES:
        movement_history.pop(0)

    return next_move
############## Main section for the Obstacle Avoidance ##############
RUN_OBSTACLE_AVOIDANCE = False # If true, run this. If false, skip it

while RUN_OBSTACLE_AVOIDANCE:
        
    # Request sensor data
    time.sleep(0.2)
    packet_tx = packetize("SD")
    transmit(packet_tx)
    time.sleep(0.1)  # Wait for the response
    [responses, time_rx] = receive()
    
    if responses[0]:
        print(f"At time '{time_rx}' received from Arduino Mega:\n{responses}")
        #Sensor_Readings = depacketize(responses)
    else:
        print(f"No valid response received for sensor data.")
    
    sensor_readings = ast.literal_eval(responses)
    next_move = decide_next_move(sensor_readings)
    print(next_move)
    if  next_move == 'move forward':
        packet_tx = packetize('w0:1')
    elif next_move == 'move backward':
        packet_tx = packetize('w0:-1')
    elif next_move == 'turn left':
        packet_tx = packetize('r0:5')
    elif next_move == 'rotate left':
        packet_tx = packetize('w0:3')
        transmit(packet_tx)
        time.sleep(0.2)
        packet_tx = packetize('r0:45')
    else:
        packet_tx = packetize('r0:-5')
    
    transmit(packet_tx)
    sensor_readings = []