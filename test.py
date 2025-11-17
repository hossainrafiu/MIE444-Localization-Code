import serial
from datetime import datetime
import time

ser = serial.Serial('COM8', 115200, timeout=0)
while True:
    command = input("Enter command: ")
    ser.write(f'[{command}]\n'.encode())
    print(f"Sent at: {datetime.now().strftime('%H:%M:%S:%f')}")
    while True:
        char = ser.read()
        if char:
            print(f"Received: {char.decode('ascii')} at {datetime.now().strftime('%H:%M:%S:%f')}")
            if char == b']':
                break
    while True:
        char = ser.read()
        if char:
            print(f"Received: {char.decode('ascii')} at {datetime.now().strftime('%H:%M:%S:%f')}")
            if char == b']':
                break