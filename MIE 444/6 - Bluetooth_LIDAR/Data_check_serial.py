import serial
import time
import re

# Set up the serial connection
ser = serial.Serial('COM4', 115200)
time.sleep(2)  # Wait for the serial connection to initialize

# Regular expression pattern to match lines with "Angle" and "Distance" or "No valid data"
pattern = re.compile(r"Angle:\s*(-?\d+)\s*degrees,\s*(Distance:\s*([\d.]+)\s*mm|No valid data)")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "Angle" in line:  # Ensure line contains the keyword "Angle"
                match = pattern.match(line)
                if match:
                    angle = int(match.group(1))
                    if match.group(3):  # Check if distance data is present
                        distance = float(match.group(3))
                        print(f"Data received: Angle: {angle} degrees, Distance: {distance} mm")
                    else:
                        print(f"Data received: Angle: {angle} degrees, No valid data")
            else:
                print("Incomplete data received, skipping line:", line)
except KeyboardInterrupt:
    print("Stopped")
finally:
    ser.close()


