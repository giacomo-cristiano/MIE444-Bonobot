import serial
import time

# Set up serial connection on COM5 at the appropriate baud rate (adjust if necessary)
bluetooth_serial = serial.Serial('COM5', 9600, timeout=1)

# Allow time for the connection to initialize
time.sleep(2)
print("Listening for data from Bluetooth...")

try:
    while True:
        # Check if there's any incoming data
        if bluetooth_serial.in_waiting > 0:
            # Read the response and decode to a readable format
            response = bluetooth_serial.readline().decode('utf-8').strip()
            if response:
                print(f"Received response: {response}")
            else:
                print("No data received.")

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Close the Bluetooth serial connection when done
    bluetooth_serial.close()
    print("Bluetooth connection closed.")
