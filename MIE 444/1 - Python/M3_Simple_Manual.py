import time
import serial

# Load your precomputed matrix with positions, orientations, and LiDAR readings
# Format for each entry in map_lidar_data: [x, y, theta, lidar_readings]
# Original data format as a single dictionary

bluetooth = serial.Serial("COM6",9600,timeout=5)

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

RUN_MANUAL = True
while RUN_MANUAL:

    cmd = input("Type in drive command: ")
    # drive_cmd = "[" + cmd + "]"
    send_command(cmd)

    