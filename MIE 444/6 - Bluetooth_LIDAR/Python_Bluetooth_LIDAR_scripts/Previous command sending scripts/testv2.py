import serial
import time
import ast

# Configure the Bluetooth serial connection on COM5
bluetooth_serial = serial.Serial('COM5', 9600, timeout=1)

# Allow time for the connection to initialize
time.sleep(2)
print("Connected to Bluetooth. Sending LIDAR request...")

def parse_lidar_data(data):
    try:
        # Convert the string data into a list using literal_eval
        sensor_readings = ast.literal_eval(data)
        
        # Print out each 10-degree reading
        if isinstance(sensor_readings, list) and len(sensor_readings) == 36:
            for i, distance in enumerate(sensor_readings):
                print(f"Angle {i*10}°: {distance} cm")
        else:
            print("Invalid data format received.")
    except (ValueError, SyntaxError) as e:
        print(f"Failed to parse response: {e}")

def request_lidar_data():
    # Send the "LD" command to Arduino
    bluetooth_serial.write(b"LD\n")
    print("Sent 'LD' command to Arduino to request LIDAR data.")

try:
    # Send the request for LIDAR data
    request_lidar_data()

    # Listen for a response from Arduino
    while True:
        # Read incoming Bluetooth data
        if bluetooth_serial.in_waiting > 0:
            response = bluetooth_serial.readline().decode('utf-8').strip()
            
            # Ensure that the response is wrapped in brackets for valid parsing
            if response.startswith("[") and response.endswith("]"):
                print(f"Raw response was: {response}")
                parse_lidar_data(response)
                break  # Exit loop after receiving and parsing one set of LIDAR data
            else:
                print(f"Invalid packet format: {response}")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Close the Bluetooth connection when done
    bluetooth_serial.close()
    print("Bluetooth connection closed.")
