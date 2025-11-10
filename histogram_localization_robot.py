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

from histogram_localization import HistogramLocalization
from robot_control import RobotDrive
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
        return [False]


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


def block_type_detected(sensor_readings: list) -> int:
    """
    Given a list of four distance sensor readings, determine the block type that is most likely
    being observed.
    """

    # Define thresholds for wall detection
    WALL_THRESHOLD = 180  # Distance in mm to consider a wall detected

    # Determine which walls are detected
    walls = [reading < WALL_THRESHOLD for reading in sensor_readings]

    # Map wall configurations to block types
    wall_config_to_block_type = {
        (False, False, False, False): 0,  # No walls
        (True, False, False, False): 1,  # 1 Wall
        (True, True, False, False): 2,  # 2 Adjacent Walls
        (True, True, True, False): 3,  # 3 Walls
        (True, True, True, True): 4,  # 4 Walls (Enclosed)
        (True, False, True, False): 5,  # 5 Opposite Walls
    }

    # check the detected walls against the mapping with rotations
    for rotation in range(4):
        rotated_walls = tuple(walls[(i - rotation) % 4] for i in range(4))
        if rotated_walls in wall_config_to_block_type:
            return wall_config_to_block_type[rotated_walls]

    return -1  # Unknown block type


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
except serial.SerialException:
    print(
        f"Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it."
    )

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0

MANUAL_CONTROL = True

############## Main section for the communication client ##############

robot = RobotDrive(packetize, transmit, receive)

omnidrive_mode = True
localizer = HistogramLocalization()
motionSteps = 0  # Update Localization every 4 motion steps

load_pick_up_location = [1, 1]  # (row, col)
with_load = False
unload_drop_off_location = [3, 7]  # (row, col)
updateMotion = False

localizer.print_belief_summary()
localizer.visualize_belief()

if True:
    robot.pingSensors()
    observed_block_type = block_type_detected(robot.ToFDistancesRaw)
    print(f"Detected block type: {observed_block_type}")
    localizer.update_belief(observed_block_type)
    localizer.visualize_belief()

while not MANUAL_CONTROL:
    # Pathfinding
    current_r, current_c, current_ori = localizer.get_most_likely_position()
    position_prob = localizer.get_position_probability(current_r, current_c)

    if position_prob > 0.4:
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
        if updateMotion:
            localizer.predict_motion(action)
            print("\n" + action + "\n")
            localizer.visualize_belief()
            updateMotion = False
    else:
        action = ""
        if updateMotion:
            motion = (
                "forward"
                if robot.currentFrontend == 0
                else (
                    "right"
                    if robot.currentFrontend == 1
                    else "backwards" if robot.currentFrontend == 2 else "left"
                )
            )
            localizer.predict_motion(motion)
            print("\n" + motion + "\n")
            localizer.visualize_belief()
            updateMotion = False

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

    if raw_cmd == "f":
        motionSteps += 1

    # Send the command(s)
    if raw_cmd == "f":
        robot.obstacleAvoidance()
        time.sleep(0.5)
        robot.obstacleAvoidance()
        time.sleep(0.5)
    else:
        robot.sendCommand(raw_cmd)
        time.sleep(0.5)

    ############### Histogram Localization Update ##############

    # Ping Sensors
    robot.pingSensors()
    print(f"Sensor Ping Responses: {robot.ToFDistancesRaw}")

    time.sleep(0.5)

    # Update motion steps and perform prediction step if necessary
    if (
        robot.ToFDistancesRaw[robot.currentFrontend] != 0
        and robot.ToFDistancesRaw[robot.currentFrontend] < 70
    ):
        motionSteps = 2  # Force an update if an obstacle is detected close ahead
    if motionSteps >= 2:
        observed_block_type = block_type_detected(robot.ToFDistancesRaw)
        print(f"\nDetected block type: {observed_block_type}\n")
        localizer.update_belief(observed_block_type)
        localizer.visualize_belief()
        motionSteps = 0
        updateMotion = True
    print(f"\nMotion steps since last update: {motionSteps}\n")

while MANUAL_CONTROL:
    print(
        "Commands: 'w' = obstacle avoidance, 'a' = turn left, 's' = move backward, 'd' = turn right, 'q' = rotate CCW, 'e' = rotate CW,\n"
    )
    print(
        "'l' = load/unload, 'p' = ping sensors, 'u' = update localization, 'us' = ultrasonic sensors, 'c' = centering, 'f' = move forward\n"
    )
    val = input("Enter command: ")
    if val not in ["w", "l", "p", "u", "us", "c"]:
        duration = input("Enter duration in milliseconds: ")
    if val.lower() == "w":
        robot.obstacleAvoidance()
    elif val.lower() == "l":
        robot.sendCommand("g")
    elif val.lower() == "p":
        robot.pingSensors()
        robot.pingSensors("u")
        plt.subplot(1, 2, 2)
        robot.plotSensorData(plt=plt)
    elif val.lower() == "c":
        robot.sendCommand("c")

    elif val.lower() == "f":
        robot.sendCommand(f"f{duration}")
    elif val.lower() == "a":
        robot.sendCommand(f"a{duration}")
    elif val.lower() == "s":
        robot.sendCommand(f"s{duration}")
    elif val.lower() == "d":
        robot.sendCommand(f"d{duration}")
    elif val.lower() == "q":
        robot.sendCommand(f"q{duration}")
    elif val.lower() == "e":
        robot.sendCommand(f"e{duration}")
    if val.lower() == "u":
        observed_block_type = block_type_detected(robot.ToFDistancesRaw)
        print(gameMap)
        type = input(
            f"\nDetected block type: {observed_block_type}\n Enter block type you want to update histogram localization with: "
        )
        localizer.update_belief(type)
        localizer.visualize_belief()
        movement = input("Enter movement to predict (f, l, r, b): ")
        movement_map = {
            "f": "forward",
            "b": "backward",
            "l": "left",
            "r": "right",
        }
        localizer.predict_motion(movement_map[movement])
        localizer.visualize_belief()
