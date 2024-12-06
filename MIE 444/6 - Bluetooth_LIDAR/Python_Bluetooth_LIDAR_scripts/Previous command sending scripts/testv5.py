import serial
import time

# Replace 'COM5' with your Bluetooth port
bluetooth = serial.Serial("COM5", 9600, timeout=1)

# Send the command to start LIDAR data collection
command = "[LD]\n"
bluetooth.write(command.encode())
print("Sent command:", command)

# Wait to receive the data
while True:
    if bluetooth.in_waiting > 0:
        data = bluetooth.readline().decode().strip()
        if data:
            # Split and parse the received data
            distances = data.split(",")
            distances = [float(d) if d != '-1' else None for d in distances]
            print("LIDAR Distances:", distances)
            break  # Exit loop after receiving one set of data

bluetooth.close()
