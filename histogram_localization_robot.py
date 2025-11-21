from client_communication import (
    ClientCommunication,
    PORT_SERIAL,
    BAUDRATE,
    TIMEOUT_SERIAL,
)
import serial
import matplotlib.pyplot as plt
from colorama import Fore

from histogram_localization import HistogramLocalization
from robot_control import RobotDrive
from pathfinding import PathfindingRobot

SHOW_PLOTS = False

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


def robotMoveForward():
    robot.obstacleAvoidance(ping=False)
    robot.pingSensors()
    plt.subplot(1, 2, 2)
    plt.cla()
    if SHOW_PLOTS:
        robot.plotSensorData(plt=plt)


############### Initialize ##############
### Source to display
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

############## Main section for the communication client ##############

clientCommunication = ClientCommunication(SER, False)
robot = RobotDrive(
    clientCommunication.packetize,
    clientCommunication.transmit,
    clientCommunication.receive,
)

localizer = HistogramLocalization()

load_pick_up_location = [1, 1]  # (row, col)
unload_drop_off_location = [3, 7]  # (row, col)

pathfinder = PathfindingRobot(
    load_pick_up_location, unload_drop_off_location, omnidrive=True
)


plt.figure(num=1, figsize=(12, 6), clear=True)
plt.subplot(1, 2, 1)

robot.pingSensors()
localizer.update_belief(block_type_detected(robot.ToFDistancesRaw))
localizer.print_belief_summary()
if SHOW_PLOTS:
    localizer.visualize_belief(plt, False)

while True:
    print(
        Fore.YELLOW
        + "Commands: 'f' = obstacle avoidance, 'f1' = avoid side walls, 'f2' = hug side walls"
    )
    print(
        "'l' = load/unload, 'p' = ping sensors, 'u' = update localization, 'c' = centering"
    )
    val = input(Fore.CYAN + "Enter command: ")
    if val.lower() == "l":
        robot.detectLoad()
    if val.lower() == "ul":
        robot.dropLoad()

    elif val.lower() == "f":
        robot.obstacleAvoidanceContinuous(duration=10000)
        
    elif val.lower() == "f1":
        robot.avoidSideWalls()
    elif val.lower() == "f2":
        robot.hugSideWalls()

    elif val.lower() == "p":
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        if SHOW_PLOTS:
            robot.plotSensorData(plt=plt)
    elif val.lower() == "v":
        robot.pingLoadSensors()

    elif val.lower() == "c":
        robot.centering()
    elif val.lower() == "c1":
        robot.centreinblock()
    elif val.lower() == "r":
        robot.simpleParallelize()
    
    elif val.lower() == "=":
        SER.close()
        print(Fore.WHITE + "Serial connection closed.")
        break

    elif val.lower() == "w":
        if robot.currentFrontend != 0:
            robot.changeFrontEnd(0)
        robotMoveForward()
    elif val.lower() == "a":
        if robot.currentFrontend != 3:
            robot.changeFrontEnd(3)
        robotMoveForward()
    elif val.lower() == "s":
        if robot.currentFrontend != 2:
            robot.changeFrontEnd(2)
        robotMoveForward()
    elif val.lower() == "d":
        if robot.currentFrontend != 1:
            robot.changeFrontEnd(1)
        robotMoveForward()
    elif val.lower() == "u":
        # Movement Update
        localizer.predict_motion(robot.currentFrontend)
        plt.subplot(1, 2, 1)
        plt.cla()
        if SHOW_PLOTS:
            localizer.visualize_belief(plt, False)
        # Sensor Update
        robot.pingSensors()
        observed_block_type = block_type_detected(robot.ToFDistancesRaw)
        print(f"Observed block type: {observed_block_type}")
        localizer.update_belief(int(observed_block_type))
        plt.subplot(1, 2, 1)
        plt.cla()
        if SHOW_PLOTS:
            localizer.visualize_belief(plt, False)
        robot.pauseInCenter = False

    elif val.lower() == "m":
        robot.changeSpeeds() # Gives current Speeds
        print("New Speeds:")
        robot.changeSpeeds(input("Motor1: "), input("Motor2: "), input("Motor3: "), input("Motor4: "))
    elif val.lower() == "*":
        try:
            SER.close()
            SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
            print(f"Connected to {SOURCE} at {BAUDRATE} bps.")
            clientCommunication.newSerial(SER)
        except serial.SerialException:
            print(
                f"Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it."
            )
    else:
        print(Fore.MAGENTA + "Sending Command to Robot: " + val)
        robot.sendCommand(val)
