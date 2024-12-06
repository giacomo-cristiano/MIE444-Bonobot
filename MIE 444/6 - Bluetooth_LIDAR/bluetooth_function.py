import serial
import time

def main():
    # Replace 'COM5' with the COM port to which your Bluetooth module is connected
    bluetooth_port = 'COM6'
    baud_rate = 9600  # Match the baud rate in your Arduino code

    try:
        # Initialize serial connection
        bluetooth = serial.Serial(bluetooth_port, baud_rate, timeout=1)
        print(f"Connected to Bluetooth module on {bluetooth_port}.")

        # Wait a moment for the connection to stabilize
        time.sleep(2)

        while True:
            if bluetooth.in_waiting > 0:
                # Read incoming data
                data = bluetooth.readline().decode('utf-8').strip()
                if data:
                    print(f"Received: {data}")
                    # Echo back the received data (optional)
                    bluetooth.write(f"Echo: {data}\n".encode('utf-8'))

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        if 'bluetooth' in locals():
            bluetooth.close()
            print("Bluetooth connection closed.")

if __name__ == "__main__":
    main()
