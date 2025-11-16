from client_communication import *
import matplotlib.pyplot as plt

from histogram_localization import HistogramLocalization
from robot_control import RobotDrive
from pathfinding import PathfindingRobot


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


def block_type_detected_V2(readings: list) -> int:
    WALL_THRESHOLD = 100
    walls = [0, 0, 0, 0]
    count = 0
    if readings[0] < WALL_THRESHOLD:
        walls[0] = 1
    if readings[1] < WALL_THRESHOLD:
        walls[1] = 1
    if readings[2] < WALL_THRESHOLD:
        walls[2] = 1
    if readings[3] < WALL_THRESHOLD:
        walls[3] = 1
    for i in range(4):
        count += walls[i]

    if count == 2:
        for i in range(4):
            if walls[i] == 1:
                if walls[i - 1] == 1:
                    return 2
        return 5
    else:
        return count


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

clientCommunication = ClientCommunication(SER)
robot = RobotDrive(
    clientCommunication.packetize,
    clientCommunication.transmit,
    clientCommunication.receive,
)

omnidrive_mode = True
localizer = HistogramLocalization()

load_pick_up_location = [1, 1]  # (row, col)
with_load = False
unload_drop_off_location = [3, 7]  # (row, col)

pathfinder = PathfindingRobot(
    load_pick_up_location, unload_drop_off_location, omnidrive=omnidrive_mode
)


plt.figure(num=1, figsize=(12, 6), clear=True)
plt.subplot(1, 2, 1)

localizer.print_belief_summary()
localizer.visualize_belief(plt, False)

while True:
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
        robot.obstacleAvoidance(ping=False)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "a":
        if robot.currentFrontend != 3:
            robot.sendCommand("r3")
        robot.obstacleAvoidance(ping=False)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "s":
        if robot.currentFrontend != 2:
            robot.sendCommand("r2")
        robot.obstacleAvoidance(ping=False)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "d":
        if robot.currentFrontend != 1:
            robot.sendCommand("r1")
        robot.obstacleAvoidance(ping=False)
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)
    elif val.lower() == "u":
        robot.pingSensors()
        observed_block_type = block_type_detected(robot.ToFDistancesRaw)
        print(f"Observed block type: {observed_block_type}")
        # print(gameMap)
        # block_type = input(
        #     f"\nDetected block type: {observed_block_type}\n Enter block type you want to update histogram localization with: "
        # )
        localizer.update_belief(int(observed_block_type))
        plt.subplot(1, 2, 1)
        plt.cla()
        localizer.visualize_belief(plt, False)
        movement = input("Enter movement to predict (f, l, r, b): ")
        movement_map = {
            "w": "forward",
            "s": "backward",
            "a": "left",
            "d": "right",
        }
        localizer.predict_motion(movement_map[movement])
        plt.subplot(1, 2, 1)
        plt.cla()
        localizer.visualize_belief(plt, False)
