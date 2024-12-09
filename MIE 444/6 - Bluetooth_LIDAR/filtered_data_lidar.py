import serial
import time
import re
import matplotlib.pyplot as plt
import numpy as np

# Set up the serial connection
ser = serial.Serial('COM10', 115200)
time.sleep(2)  # Wait for the serial connection to initialize

# Regular expression pattern to match lines with "Angle" and "Distance"
pattern = re.compile(r"Angle:\s*(-?\d+)\s*degrees,\s*Distance:\s*([\d.]+)\s*mm")

# Initialize lists with NaN placeholders
angles = [np.nan] * 36
distances = [np.nan] * 36
start_time = time.time()  # Record the start time

try:
    while True:
        # Set a timeout of, say, 10 seconds to collect data
        if time.time() - start_time > 10:  
            print("Timeout reached. Proceeding with available data.")
            break

        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "Angle" in line and "Distance" in line:  # Ensure line contains both keywords
                match = pattern.match(line)
                if match:
                    angle = int(match.group(1))
                    distance = float(match.group(2))
                    print(f"Data received: Angle: {angle} degrees, Distance: {distance} mm")
                    
                    # Calculate the index based on angle (assuming angles are multiples of 10)
                    index = angle // 10
                    if 0 <= index < 36:
                        angles[index] = angle
                        distances[index] = distance
                    
            else:
                print("Incomplete data received, skipping line:", line)
except KeyboardInterrupt:
    print("Stopped")
finally:
    ser.close()

# Remove NaN values for plotting
angles_clean = np.array(angles)[~np.isnan(angles)]
distances_clean = np.array(distances)[~np.isnan(distances)]
angles_rad = np.radians(angles_clean)

# Plot the polar plot
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')
ax.plot(angles_rad, distances_clean, marker='o', linestyle='-')

# Customize plot
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_title("LIDAR Distance Readings at 10-degree Increments")

# Show the plot
plt.show()
