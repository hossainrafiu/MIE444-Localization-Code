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
        # cmd_list = [item.split(":", 1) for item in data]

        # # Make sure this list is formatted in the expected manner
        # for cmd_single in cmd_list:
        #     match len(cmd_single):
        #         case 0:
        #             cmd_single.append("")
        #             cmd_single.append("")
        #         case 1:
        #             cmd_single.append("")
        #         case 2:
        #             pass
        #         case _:
        #             pass
        #             # cmd_single = cmd_single[0:2]

        # return cmd_list
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


def cordsToGrid(x: float, y: float, theta: float) -> tuple:
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


def shift_sensor_readings(sensor_readings: list, current_frontend: int) -> list:
    """Shift sensor readings based on current frontend orientation."""
    # current_frontend: 0: forward, 1: right, 2: backward, 3: left
    shifted_readings = [0, 0, 0, 0]
    for i in range(4):
        shifted_readings[i] = sensor_readings[(i + current_frontend) % 4]
    return shifted_readings


############## Constant Definitions Begin ##############
### Network Setup ###
HOST = "127.0.0.1"  # The server's hostname or IP address
PORT_TX = 61200  # The port used by the *CLIENT* to receive
PORT_RX = 61201  # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600  # Baudrate in bps
PORT_SERIAL = "COM8"  # COM port identification
TIMEOUT_SERIAL = 1  # Serial port timeout, in seconds

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

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0


############## Main section for the communication client ##############

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

omnidrive_mode = True
last_sensor_readings = [0, 0, 0, 0]
current_frontend = 0  # 0: forward, 1: right, 2: backward, 3: left

time.sleep(5)  # Wait a bit for everything to start up

fig, ax = plt.subplots()
pf.plot_particles(ax)
plt.ion()
plt.show()
plt.pause(2)

# Ping Sensors
raw_cmd = "p"
packet_tx = packetize(raw_cmd)
if packet_tx:
    transmit(packet_tx)
while True:
    [responses, time_rx] = receive()
    if responses[0] == raw_cmd:
        continue
    if responses[0] == "+" or responses[0] is not False:
        break
print(f"Sensor Ping Responses at {time_rx}: {responses}")
last_sensor_readings = [float(responses[i]) + 2.5 for i in range(len(responses) - 1)]
current_frontend = int(responses[-1])

while True:
    # Pathfinding
    x, y, theta = pf.estimate_position()
    confidence = pf.get_confidence()
    print(f"Estimated Position: x={x}, y={y}, theta={theta}, confidence={confidence}")

    if confidence > 50:
        current_c, current_r, current_ori = cordsToGrid(x, y, theta)
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

    if action == "":
        print(
            "Warning: Low confidence in current position estimate. Just move forward."
        )
        raw_cmd = "f"
    if action == "forward":
        if current_frontend == 0:
            raw_cmd = "f"
        else:
            raw_cmd = "r0"
    if action == "right":
        if current_frontend == 1:
            raw_cmd = "f"
        else:
            raw_cmd = "r1"
    if action == "backward":
        if current_frontend == 2:
            raw_cmd = "f"
        else:
            raw_cmd = "r2"
    if action == "left":
        if current_frontend == 3:
            raw_cmd = "f"
        else:
            raw_cmd = "r3"
    if action == "wait":
        raw_cmd = "h"
    if action == "pickup" or action == "dropoff":
        raw_cmd = "l"
        with_load = not with_load

    # Send the command
    packet_tx = packetize(raw_cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive the response
    while True:
        [responses, time_rx] = receive()
        if responses[0] == raw_cmd:
            continue
        if responses[0] == "+" or responses[0] is not False:
            break
    print(f"Command Response at {time_rx}: {responses}")

    time.sleep(1)  # Wait a bit before requesting all sensor readings

    ############### Particle Filter Update ##############

    if raw_cmd == "f":
        delta_x = 3 if current_frontend == 0 else -3 if current_frontend == 2 else 0
    else:
        delta_x = 0

    if raw_cmd == "f":
        delta_y = -3 if current_frontend == 1 else 3 if current_frontend == 3 else 0
    else:
        delta_y = 0

    delta_theta = 0
    if raw_cmd.startswith("r") and not omnidrive_mode:
        delta_y = 0
        delta_x = 0
        new_frontend = int(raw_cmd[1])
        turn_steps = (new_frontend - current_frontend) % 4
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
    raw_cmd = "p"
    packet_tx = packetize(raw_cmd)
    if packet_tx:
        transmit(packet_tx)
    while True:
        [responses, time_rx] = receive()
        if responses[0] == raw_cmd:
            continue
        if responses[0] == "+" or responses[0] is not False:
            break
    print(f"Sensor Ping Responses at {time_rx}: {responses}")
    major_change_detected = []
    for i in range(len(responses) - 1):
        if (
            abs(float(responses[i]) / 25.4 - last_sensor_readings[i]) > 5.0
            and last_sensor_readings[i] != 0
        ):
            major_change_detected.append(i)
        last_sensor_readings[i] = float(responses[i]) / 25.4 + 3.0
    current_frontend = responses[-1]

    if major_change_detected:
        print(f"Major change detected on sensors: {major_change_detected}")
        # send forward commands to avoid wall edge
        for _ in range(len(major_change_detected)):
            raw_cmd = "f"
            packet_tx = packetize(raw_cmd)
            if packet_tx:
                transmit(packet_tx)
            while True:
                [responses, time_rx] = receive()
                if responses[0] == raw_cmd:
                    continue
                if responses[0] == "+" or responses[0] is not False:
                    break
            print(f"Command Response at {time_rx}: {responses}")
            pf.move_particles(
                1 if current_frontend == 0 else -1 if current_frontend == 2 else 0,
                -1 if current_frontend == 1 else 1 if current_frontend == 3 else 0,
                0,
            )
            # pf.move_particles_improved(1 if current_frontend == 0 else -1 if current_frontend == 2 else 0,
            #                   -1 if current_frontend == 1 else 1 if current_frontend == 3 else 0,
            #                   0)

    sensor_readings = (
        shift_sensor_readings(last_sensor_readings, current_frontend)
        if not omnidrive_mode
        else last_sensor_readings
    )
    print(sensor_readings)

    # Swap sensor readings 1 and 3 for correct orientation
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
