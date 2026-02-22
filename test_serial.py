import serial
import time
import threading

# --- Configuration ---
# On Mac, ports look like /dev/tty.usbmodem12345 or /dev/tty.usbserial
# Run 'ls /dev/tty.*' in terminal to find the correct one.
SERIAL_PORT = '/dev/tty.usbmodem1101' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Error: {e}")
    exit()

def read_loop():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"Received from Teensy: {line}")
        except Exception as e:
            print(f"Error reading: {e}")
            break

# Run loop
try:
    read_loop()
except KeyboardInterrupt:
    print("\nClosing connection...")
    ser.close()