"""
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import math
import socket
import time
from datetime import datetime
import serial
import matplotlib.pyplot as plt

from localization import ParticleFilter
from pathfinding import *
from robot_control import RobotDrive

verboseConsole = False  # If true, print all transmitted and received data to console


# Wrapper functions
def transmit(data):
    """Selects whether to use serial or tcp for transmitting."""
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)


def receive():
    """Selects whether to use serial or tcp for receiving."""
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()


# TCP communication functions
def transmit_tcp(data):
    """Send a command over the TCP connection."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode("ascii"))
        except (ConnectionRefusedError, ConnectionResetError):
            print("Tx Connection was refused or reset.")
        except TimeoutError:
            print("Tx socket timed out.")
        except EOFError:
            print("\nKeyboardInterrupt triggered. Closing...")


def receive_tcp():
    """Receive a reply over the TCP connection."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode("ascii")
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print("Rx connection was refused or reset.")
        except TimeoutError:
            print("Response not received from robot.")


# Serial communication functions
def transmit_serial(data):
    """Transmit a command over a serial connection."""
    clear_serial()
    SER.write(data.encode("ascii"))


def receive_serial():
    """Receive a reply over a serial connection."""

    start_time = time.time()
    response_raw = ""
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode("ascii")
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    if verboseConsole:
        print(f"Raw response was: {response_raw}")

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]


def clear_serial(delay_time: float = 0):
    """Wait some time (delay_time) and then clear the serial buffer."""
    if SER.in_waiting:
        time.sleep(delay_time)
        if verboseConsole:
            print(f"Clearing Serial... Dumped: {SER.read(SER.in_waiting)}")


# Packetization and validation functions
def depacketize(data_raw: str):
    """
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    """

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if start >= 0 and end >= start:
        data = (
            data_raw[start + 1 : end].replace(f"{FRAMEEND}{FRAMESTART}", ",").split(",")
        )
        return data

    else:
        return [[False, ""]]


def packetize(data: str):
    """
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    """

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, "\n"]
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False


def response_string(cmds: str, responses_list: list):
    """
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    """
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(":")[0] for item in cmds.split(",")]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ""
    sgn = ""
    chk = ""
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = "="
            chk = "✓"
        else:
            sgn = "!="
            chk = "X"

        out_string = out_string + (
            f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n'
        )

    return out_string


def validate_responses(cmd_list: list, responses_list: list):
    """
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    """
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid


def cords_to_grid(x: float, y: float, theta: float) -> tuple:
    """Convert real-world coordinates to grid coordinates and orientation."""
    # Assuming each grid cell is 12x12 inches and the origin (0,0) is at the center of cell (0,0)
    col, row, orientation = 0, 0, 0
    if x >= 0 and x < 12:
        col = 0
    elif x >= 12 and x < 24:
        col = 1
    elif x >= 24 and x < 36:
        col = 2
    elif x >= 36 and x < 48:
        col = 3
    elif x >= 48 and x < 60:
        col = 4
    elif x >= 60 and x < 72:
        col = 5
    elif x >= 72 and x < 84:
        col = 6
    elif x >= 84 and x < 96:
        col = 7
    if y >= 0 and y < 12:
        row = 3
    elif y >= 12 and y < 24:
        row = 2
    elif y >= 24 and y < 36:
        row = 1
    elif y >= 36 and y < 48:
        row = 0
    theta = theta % (2 * math.pi)
    if (theta >= 7 * math.pi / 4 and theta < 2 * math.pi) or (
        theta >= 0 and theta < math.pi / 4
    ):
        orientation = 1
    elif theta >= math.pi / 4 and theta < 3 * math.pi / 4:
        orientation = 0
    elif theta >= 3 * math.pi / 4 and theta < 5 * math.pi / 4:
        orientation = 3
    elif theta >= 5 * math.pi / 4 and theta < 7 * math.pi / 4:
        orientation = 2

    return (col, row, orientation)


def shift_sensor_readings(_sensor_readings: list, current_frontend: int) -> list:
    """Shift sensor readings based on current frontend orientation."""
    # current_frontend: 0: forward, 1: right, 2: backward, 3: left
    shifted_readings = [0, 0, 0, 0]
    for i in range(4):
        shifted_readings[i] = _sensor_readings[(i + current_frontend) % 4]
    return shifted_readings


def get_delta() -> tuple:
    """Get the delta x, y, and theta based on the current frontend and action."""
    delta_x = (
        3 if robot.currentFrontend == 0 else -3 if robot.currentFrontend == 2 else 0
    )
    delta_y = (
        -3 if robot.currentFrontend == 1 else 3 if robot.currentFrontend == 3 else 0
    )
    delta_theta = 0
    return (delta_x, delta_y, delta_theta)


## TESTING

# robot = RobotDrive(packetize, transmit, receive)
# robot.ToFDistancesRaw = [100, 200, 300, 400]
# robot.USDistancesRaw = [200, 200, 200, 200, 200, 200, 200, 200]
# robot.plotSensorData(plt=plt)

############## Constant Definitions Begin ##############
### Network Setup ###
HOST = "127.0.0.1"  # The server's hostname or IP address
PORT_TX = 61200  # The port used by the *CLIENT* to receive
PORT_RX = 61201  # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 115200  # Baudrate in bps
PORT_SERIAL = "COM8"  # COM port identification
TIMEOUT_SERIAL = 3  # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = "["
FRAMEEND = "]"
CMD_DELIMITER = ","

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = False


############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = "SimMeR"
else:
    SOURCE = "serial device " + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
    print(f"Connected to {SOURCE} at {BAUDRATE} bps.")
except serial.SerialException:
    print(
        f"Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it."
    )
    while True:
        pass

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0.1

MANUAL_CONTROL = True

############## Main section for the communication client ##############

robot = RobotDrive(packetize, transmit, receive)

pf = ParticleFilter(
    num_particles=100,
    initial_multiplier=10,
    sensor_range=20,
    sensor_noise=1.0,
    motion_noise=0.5,
)

load_pick_up_location = [1, 1]  # (row, col)
with_load = False
unload_drop_off_location = [3, 7]  # (row, col)
prev_action = ""

omnidrive_mode = True

time.sleep(5)  # Wait a bit for everything to start up

plt.figure(1, figsize=(8, 6), clear=True)
plt.subplot(1, 2, 1)
ax = plt.gca()
pf.plot_particles(ax)
plt.ion()
plt.show()

# Ping Sensors
# robot.pingSensors()
# robot.sendCommand("c")  # Centering

while not MANUAL_CONTROL:
    # Pathfinding
    x, y, theta = pf.estimate_position()
    confidence = pf.get_confidence()
    print(f"Estimated Position: x={x}, y={y}, theta={theta}, confidence={confidence}")

    if confidence > 50:
        current_c, current_r, current_ori = cords_to_grid(x, y, theta)
        print(f"Grid Position: row={current_r}, col={current_c}, ori={current_ori}")
        target_rc = goal_from_state(
            with_load, load_pick_up_location, unload_drop_off_location
        )
        action, path = next_action_to_objective(
            int(current_r),
            int(current_c),
            int(current_ori),
            with_load,
            omnidrive=omnidrive_mode,
            pickup=load_pick_up_location,
            dropoff=unload_drop_off_location,
        )
        print(f"Next action: {action}")
        print(path)
    else:
        action = ""

    # Avoid Wall Corners
    if prev_action != "" and action != prev_action:
        robot.obstacleAvoidance()  # To do ping sensors and avoid corners when turning
    prev_action = action

    raw_cmd = "f"
    if action == "":
        print(
            "Warning: Low confidence in current position estimate. Just move forward."
        )
        raw_cmd = "f"
    if action == "forward":
        if robot.currentFrontend == 0:
            raw_cmd = "f"
        else:
            raw_cmd = "r0"
    if action == "right":
        if robot.currentFrontend == 1:
            raw_cmd = "f"
        else:
            raw_cmd = "r1"
    if action == "backward":
        if robot.currentFrontend == 2:
            raw_cmd = "f"
        else:
            raw_cmd = "r2"
    if action == "left":
        if robot.currentFrontend == 3:
            raw_cmd = "f"
        else:
            raw_cmd = "r3"
    if action == "wait":
        raw_cmd = "h"
    if action == "pickup" or action == "dropoff":
        raw_cmd = "l"
        with_load = not with_load

    # Send the command
    if raw_cmd == "f":
        robot.obstacleAvoidance()
        time.sleep(0.5)
    else:
        robot.sendCommand(raw_cmd)
        time.sleep(0.5)

    ############### Particle Filter Update ##############

    if raw_cmd == "f":
        delta_x = (
            3 if robot.currentFrontend == 0 else -3 if robot.currentFrontend == 2 else 0
        )
    else:
        delta_x = 0

    if raw_cmd == "f":
        delta_y = (
            -3 if robot.currentFrontend == 1 else 3 if robot.currentFrontend == 3 else 0
        )
    else:
        delta_y = 0

    delta_theta = 0
    if raw_cmd.startswith("r") and not omnidrive_mode:
        delta_y = 0
        delta_x = 0
        new_frontend = int(raw_cmd[1])
        turn_steps = (new_frontend - robot.currentFrontend) % 4
        if turn_steps == 1:
            delta_theta = math.pi / 2
        elif turn_steps == 2:
            delta_theta = math.pi
        elif turn_steps == 3:
            delta_theta = -math.pi / 2

    print(f"Motion command: Δx={delta_x}, Δy={delta_y}, Δθ={delta_theta}")
    pf.move_particles(delta_x, delta_y, delta_theta)
    # pf.move_particles_improved(delta_x, delta_y, delta_theta)

    # Plot particles and estimated position
    estimated_pos = pf.estimate_position()
    print(f"Estimated Position: {estimated_pos}")
    plt.title("Particle Filter After Resampling")
    pf.plot_particles(ax)
    pf.plot_estimated_position(ax, estimated_pos)
    plt.show()
    plt.pause(1)

    # Ping Sensors
    robot.pingSensors()
    robot.avoidCornersIfTurning()

    sensor_readings = (
        shift_sensor_readings(robot.ToFDistancesRaw, robot.currentFrontend)
        if not omnidrive_mode
        else robot.ToFDistancesRaw
    )
    # Convert mm to inches
    sensor_readings = [x / 25.4 for x in sensor_readings]
    print(sensor_readings)

    # Swap sensor readings 1 and 3 for correct orientation in particle filter
    sensor_readings[1], sensor_readings[3] = sensor_readings[3], sensor_readings[1]

    print(f"Sensor readings (inches): {sensor_readings}")

    # Particle Filter Update
    pf.update_weights_improved(sensor_readings)

    # Plot particles and estimated position
    estimated_pos = pf.estimate_position()
    print(f"Estimated Position: {estimated_pos}")
    plt.title("Particle Filter After Resampling")
    pf.plot_particles(ax)
    pf.plot_estimated_position(ax, estimated_pos)
    plt.show()
    plt.pause(1)

    # pf.resample_particles()
    pf.resample_particles_improved()

    # Plot particles and estimated position
    estimated_pos = pf.estimate_position()
    print(f"Estimated Position: {estimated_pos} num particles: {len(pf.particles)}")
    plt.title("Particle Filter After Resampling")
    pf.plot_particles(ax)
    pf.plot_estimated_position(ax, estimated_pos)
    plt.show()

while MANUAL_CONTROL:
    print(
        "Commands: 'w' = obstacle avoidance, 'wasd' = omni motion, 'yghj' = normal motion, 'q' = rotate CCW, 'e' = rotate CW,\n"
    )
    print(
        "'l' = load/unload, 'p' = ping sensors, 'u' = update localization, 'us' = ultrasonic sensors, 'c' = centering\n"
    )
    val = input("Enter command: ")
    # duration = ""
    # if val not in ["w", "l", "p", "u", "us", "c", "o", "=", "z","x", "w", "a", "s", "d"]:
    #     duration = input("Enter duration in milliseconds: ")
    if val.lower() == "l":
        robot.sendCommand("g")
    elif val.lower() == "p":
        robot.pingSensors()
        # robot.pingSensors("u2")
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "u":
        # delta_x = input("Enter delta x in inches: ")
        # delta_y = input("Enter delta y in inches: ")
        delta_x, delta_y, delta_theta = get_delta()
        print(f"Moving particles by Δx={delta_x}, Δy={delta_y}, Δθ={delta_theta}")
        pf.move_particles(delta_x, delta_y, delta_theta)
        # robot.pingSensors()
        sensor_readings = [
            x / 25.4 for x in robot.ToFDistancesRaw
        ]  # Convert mm to inches
        sensor_readings[1], sensor_readings[3] = sensor_readings[3], sensor_readings[1]
        pf.update_weights_improved(sensor_readings)
        pf.resample_particles_improved()
        plt.subplot(1, 2, 1)
        ax = plt.gca()
        pf.plot_particles(ax)
        pf.plot_estimated_position(ax, pf.estimate_position())
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "c":
        robot.sendCommand("c")
    elif val.lower() == "o":
        robot.sendCommand("o")
    elif val.lower() == "z":
        robot.sendCommand("z")
    elif val.lower() == "x":
        robot.sendCommand("x")

    elif val.lower() == "y":
        robot.sendCommand("f200")
    elif val.lower() == "g":
        robot.sendCommand("a200")
    elif val.lower() == "h":
        robot.sendCommand("s200")
    elif val.lower() == "j":
        robot.sendCommand("d200")
    elif val.lower() == "q":
        robot.sendCommand("q200")
    elif val.lower() == "e":
        robot.sendCommand("e200")
    elif val.lower() == "=":
        SER.close()
        break
    elif val.lower() == "w":
        if robot.currentFrontend != 0:
            robot.sendCommand("r0")
        else:
            robot.obstacleAvoidance(ping=False, duration=900)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "a":
        if robot.currentFrontend != 3:
            robot.sendCommand("r3")
        else:
            robot.obstacleAvoidance(ping=False, duration=800)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "s":
        if robot.currentFrontend != 2:
            robot.sendCommand("r2")
        else:
            robot.obstacleAvoidance(ping=False, duration=900)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "d":
        if robot.currentFrontend != 1:
            robot.sendCommand("r1")
        else:
            robot.obstacleAvoidance(ping=False, duration=800)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
