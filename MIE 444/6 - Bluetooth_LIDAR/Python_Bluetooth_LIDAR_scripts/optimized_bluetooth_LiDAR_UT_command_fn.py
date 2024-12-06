import serial
import time

# Replace 'COM5' with your Bluetooth port
bluetooth = serial.Serial("COM6", 9600, timeout=1)

def send_command(command):
    bluetooth.write((command + "\n").encode())
    print(f"Sent command: {command}")

    # Wait briefly for Arduino to process the command
    time.sleep(0.2)

    # Check for acknowledgment
    ack = bluetooth.readline().decode().strip()
    if ack == "ACK":
        print("Command acknowledged by Arduino.")
    else:
        print("No acknowledgment received. Retrying...")
        time.sleep(0.2)
        bluetooth.write((command + "\n").encode())
        ack = bluetooth.readline().decode().strip()
        if ack == "ACK":
            print("Command acknowledged after retry.")
        else:
            print("Failed to receive acknowledgment. Please check connection.")

    # Retry receiving data until a response is received
    retries = 5
    for _ in range(retries):
        data = bluetooth.readline().decode().strip()
        if data:
            if command == "[LD]":
                distances = data.split(",")
                distances = [float(d) if d != '-1' else None for d in distances]
                print("LIDAR Distances:", distances)
                break
            elif command == "[UD]":
                sensor_values = data.split(",")
                sensor_values = [float(val) for val in sensor_values]
                print("Ultrasonic Sensor Distances (inches):", sensor_values)
                break
        else:
            print("No data received yet. Retrying...")
            time.sleep(0.2)  # Wait a bit before retrying

# Main loop to allow manual command entry
while True:
    command = input("Enter command ([LD] for LIDAR or [UD] for Ultrasonic, or 'exit' to quit): ").strip()
    if command == "exit":
        break
    elif command in ["[LD]", "[UD]"]:
        send_command(command)
    else:
        print("Invalid command. Please enter [LD], [UD], or 'exit'.")

bluetooth.close()
