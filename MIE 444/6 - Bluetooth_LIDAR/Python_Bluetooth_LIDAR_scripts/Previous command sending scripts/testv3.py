import serial
import time

# Set up Bluetooth connection on COM5 (adjust as needed)
bluetooth_serial = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)
print("Connected to Bluetooth.")

def send_command(command):
    # Send command to Arduino
    bluetooth_serial.write((command + "\n").encode('ascii'))
    print(f"Sent command: {command}")

def read_response():
    # Read response from Arduino
    response = bluetooth_serial.readline().decode('utf-8').strip()
    print(f"Response: {response}")

try:
    # Send the LD command
    send_command("LD")
    time.sleep(0.1)  # Allow time for response
    read_response()

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    bluetooth_serial.close()
    print("Bluetooth connection closed.")
