
import serial
import time
import re
import matplotlib.pyplot as plt
import numpy as np

# Set up the serial connection
ser = serial.Serial('COM4', 115200)
time.sleep(2)  # Wait for the serial connection to initialize

# Regular expression pattern to match lines with "Angle" and "Distance"
pattern = re.compile(r"Angle:\s*(-?\d+)\s*degrees,\s*Distance:\s*([\d.]+)\s*mm")

# Initialize lists to store angles and distances
angles = []
distances = []
try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "Angle" in line and "Distance" in line:  # Ensure line contains both keywords
                match = pattern.match(line)
                if match:
                    angle = int(match.group(1))
                    distance = float(match.group(2))
                    print(f"Data received: Angle: {angle} degrees, Distance: {distance} mm")
                    
                    # Append to lists
                    angles.append(angle)
                    distances.append(distance)
                    
                    # Break loop after 36 readings (0 to 350 degrees)
                    if len(angles) == 36:
                        break
            else:
                print("Incomplete data received, skipping line:", line)
except KeyboardInterrupt:
    print("Stopped")

finally:
    ser.close()

# Convert angles to radians for polar plot
angles_rad = np.radians(angles)

# Plot the polar plot
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')
ax.plot(angles_rad, distances, marker='o', linestyle='-')

# Customize plot
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_title("LIDAR Distance Readings at 10-degree Increments")

# Show the plot
plt.show()
