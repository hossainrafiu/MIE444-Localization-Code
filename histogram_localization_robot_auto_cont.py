import matplotlib.pyplot as plt
from colorama import Fore
import serial
import time

from client_communication import (
    ClientCommunication,
    PORT_SERIAL,
    BAUDRATE,
    TIMEOUT_SERIAL,
)
from histogram_localization import HistogramLocalization
from robot_control import RobotDrive
from pathfinding import PathfindingRobot

############### Initialize ##############
### Source to display
SOURCE = "serial device " + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
    print(Fore.GREEN + f"Connected to {SOURCE} at {BAUDRATE} bps.")
except serial.SerialException:
    print(
        Fore.RED
        + f"Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it."
    )
    while True:
        pass

############## Main section for the communication client ##############

clientCommunication = ClientCommunication(SER, False)
robot = RobotDrive(
    clientCommunication.packetize,
    clientCommunication.transmit,
    clientCommunication.receive,
)
localizer = HistogramLocalization()

load_pick_up_location = (1, 1)  # (row, col)
unload_drop_off_location = (3, 7)  # (row, col)

pathfinder = PathfindingRobot(
    load_pick_up_location, unload_drop_off_location, carrying_load=False
)


def block_type_detected(readings: list) -> int:
    WALL_THRESHOLD = 180
    walls = [1 if reading < WALL_THRESHOLD else 0 for reading in readings]
    if sum(walls) == 2:
        for i in range(4):
            if walls[i] == 1:
                if walls[i - 1] == 1:
                    return 2
        return 5
    else:
        return sum(walls)


def reset_histogram_localization(_localizer=localizer, _robot=robot):
    _localizer.reset_belief()

    _robot.pingSensors()
    plt.subplot(1, 2, 2)
    _robot.plotSensorData(plt=plt)

    _observed_block_type = block_type_detected(_robot.ToFDistancesRaw)
    print(Fore.MAGENTA + f"Detected block type: {_observed_block_type}")
    _localizer.update_belief(_observed_block_type)

    plt.subplot(1, 2, 1)
    _localizer.visualize_belief(plt, False)


def robotMoveForward(direction=0):
    if robot.currentFrontend != direction:
        robot.changeFrontEnd(direction)
        robot.pingSensors()

    if robot.ToFDistances[0] > 150:
        robot.obstacleAvoidanceContinuous()
        return True

    else:
        print(Fore.RED + "ERROR: Obstacle detected ahead, cannot move forward.")
        reset_histogram_localization()
        return False


plt.figure(num=1, figsize=(12, 6), clear=True)
plt.subplot(1, 2, 1)

RUN_STARTUP_CODE = True
TRIAL_STARTUP = True
RESET_ARDUINO = False

if RESET_ARDUINO:
    robot.sendCommand("v")  # Resets Arduino
    time.sleep(5)  # Wait for Arduino to reset

if RUN_STARTUP_CODE:
    if TRIAL_STARTUP:
        robot.centering()
    reset_histogram_localization()
    robot.changeSpeeds(motor1=75, motor2=60, motor3=75, motor4=75)

updateHistogram = False
while True:
    ############### Pathfinding and Action Decision ##############
    current_r, current_c, current_ori = localizer.get_most_likely_position()
    position_prob = localizer.get_position_probability(current_r, current_c)

    if position_prob > 0.4:
        action, path = pathfinder.get_next_action_to_objective(
            current_r,
            current_c,
            current_ori,
        )
    else:
        action = ""

    print(Fore.CYAN + f"Action decided by pathfinder: {action}")
    plt.pause(0.2)

    ############### Execute Action ##############
    if action == "":
        print(
            Fore.YELLOW
            + "Warning: Low confidence in current position estimate. Just move forward."
        )
        if robot.ToFDistances[0] > 150:
            robot.obstacleAvoidanceContinuous()
            updateHistogram = True
        else:
            newFrontend = (robot.currentFrontend + 1) % 4
            robot.changeFrontEnd(newFrontend)
            robot.pingSensors()

    if action == "forward":
        updateHistogram = robotMoveForward(direction=0)

    if action == "right":
        updateHistogram = robotMoveForward(direction=1)

    if action == "backward":
        updateHistogram = robotMoveForward(direction=2)

    if action == "left":
        updateHistogram = robotMoveForward(direction=3)

    if action == "pickup":
        robot.loadingZoneSequence()
        localizer.reset_belief_in_loading_zone()
        pathfinder.set_load_status(True)

    if action == "dropoff":
        robot.dropLoadV2()

    if action == "wait":
        print("PATHFINDER GAVE WAIT COMMAND. ERROR.")

    if action == "arrived":
        print("PATHFINDER GAVE ARRIVED COMMAND. ERROR.")

    ############### Histogram Localization Update ##############
    if updateHistogram:
        # Movement We just made
        localizer.predict_motion(robot.currentFrontend)

        # Ping Sensors and Detect Block Type
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
        block_type = block_type_detected(robot.ToFDistances)
        print(Fore.MAGENTA + f"Block Type Detected = {block_type}")

        # Update Histogram Localization with Detected Block Type
        localizer.update_belief(block_type)
        plt.subplot(1, 2, 1)
        plt.cla()
        localizer.visualize_belief(plt, False)

        plt.pause(0.2)

        updateHistogram = False
