import matplotlib.pyplot as plt
import numpy as np
import serial
import re
import time

# Set up the serial connection (adjust COM port as needed)
ser = serial.Serial('COM4', 115200)  # Replace 'COM4' with your actual port if needed
time.sleep(2)  # Wait for the serial connection to initialize

# Initialize data arrays
angles = []
distances = []

# Set up the plot
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_theta_zero_location("N")  # Set 0 degrees to point north
ax.set_theta_direction(-1)       # Set direction of angle to clockwise
ax.set_title("Real-Time LiDAR Data Visualization", va='bottom')

# Regular expression pattern to match "Angle: X degrees, Distance: Y mm"
pattern = re.compile(r"Angle:\s*(-?\d+)\s*degrees,\s*Distance:\s*([\d.]+)\s*mm")

def update_plot():
    # Convert angles to radians for the polar plot
    radians = [np.deg2rad(angle) for angle in angles]
    
    # Clear and replot the data
    ax.clear()
    ax.plot(radians, distances, marker='o', linestyle='-', color='b')
    ax.set_rmax(max(distances) * 1.1 if distances else 500)  # Set radial limit slightly above max distance
    ax.set_rticks([50, 150, 250, 350, 450])  # Radial distance ticks
    ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    plt.draw()
    plt.pause(0.01)

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            match = pattern.match(line)
            if match:
                angle = int(match.group(1))
                distance = float(match.group(2))

                # Update angles and distances arrays
                if 0 <= angle <= 350 and angle % 10 == 0:
                    # Update or append new data point
                    index = angle // 10
                    if index < len(angles):
                        distances[index] = distance  # Update existing angle
                    else:
                        angles.append(angle)
                        distances.append(distance)

                    # Plot data when a full set of 10-degree increments is reached
                    if len(angles) == 36:
                        update_plot()

except KeyboardInterrupt:
    print("Stopping visualization.")
finally:
    ser.close()
    plt.ioff()
    plt.show()
