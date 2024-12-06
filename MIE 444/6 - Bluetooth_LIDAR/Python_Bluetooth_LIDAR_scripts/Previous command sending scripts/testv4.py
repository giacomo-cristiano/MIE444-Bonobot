import serial

# Connect to the correct Bluetooth port
bluetooth = serial.Serial("COM5", 9600, timeout=4)

while True:
    try:
        if bluetooth.in_waiting > 0:
            # Read a line of data
            data = bluetooth.readline().decode().strip()
            
            # Split the data into a list of distances
            distances = data.split(",")
            distances = [float(d) if d != '-1' else None for d in distances]
            
            print("LIDAR Distances:", distances)  # Print the received distances
            
    except Exception as e:
        print("Error:", e)
        break

bluetooth.close()
