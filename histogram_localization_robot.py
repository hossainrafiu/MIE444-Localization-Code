from client_communication import ClientCommunication, PORT_SERIAL, BAUDRATE, TIMEOUT_SERIAL
import serial
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


def robotMoveForward():
    robot.obstacleAvoidance(ping=False)
    robot.pingSensors()
    plt.subplot(1, 2, 2)
    plt.cla()
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

robot.pingSensors()
localizer.update_belief(block_type_detected(robot.ToFDistancesRaw))
localizer.print_belief_summary()
localizer.visualize_belief(plt, False)

while True:
    print(
        "Commands: 'w' = obstacle avoidance, 'wasd' = omni motion, 'yghj' = normal motion, 'q' = rotate CCW, 'e' = rotate CW,"
    )
    print(
        "'l' = load/unload, 'p' = ping sensors, 'u' = update localization, 'us' = ultrasonic sensors, 'c' = centering"
    )
    val = input("Enter command: ")
    duration = 200
    if val.lower() == "l":
        robot.detectLoad()
    if val.lower() == "ul":
        robot.dropLoad()
        
    elif val.lower() == "f":
        robot.obstacleAvoidanceContinuous(200)
    elif val.lower() == "f1":
        robot.avoidSideWalls()
    elif val.lower() == "f2":
        robot.hugSideWalls()
    

    elif val.lower() == "p":
        robot.pingSensors()
        plt.subplot(1, 2, 2)
        plt.cla()
        robot.plotSensorData(plt=plt)

    elif val.lower() == "c":
        robot.sendCommand("c")
        # robot.centreinblock()
        # robot.checkCentering()
    elif val.lower() == "o":
        robot.sendCommand("o")
    elif val.lower() == "z":
        robot.sendCommand("z")
    elif val.lower() == "x":
        robot.sendCommand("x")

    elif val.lower() == "y":
        robot.sendCommand(f"f{duration}")
    elif val.lower() == "g":
        robot.sendCommand(f"a{duration}")
    elif val.lower() == "h":
        robot.sendCommand(f"s{duration}")
    elif val.lower() == "j":
        robot.sendCommand(f"d{duration}")
    elif val.lower() == "q":
        robot.sendCommand(f"q{duration}")
    elif val.lower() == "e":
        robot.sendCommand(f"e{duration}")
    elif val.lower() == "=":
        SER.close()
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
        movement = robot.currentFrontend
        movement_map = {
            0: "forward",
            1: "right",
            2: "backward",
            3: "left",
        }
        localizer.predict_motion(movement_map[movement])
        plt.subplot(1, 2, 1)
        plt.cla()
        localizer.visualize_belief(plt, False)
        # Sensor Update
        robot.pingSensors()
        observed_block_type = block_type_detected(robot.ToFDistancesRaw)
        print(f"Observed block type: {observed_block_type}")
        localizer.update_belief(int(observed_block_type))
        plt.subplot(1, 2, 1)
        plt.cla()
        localizer.visualize_belief(plt, False)
        robot.pauseInCenter = False
