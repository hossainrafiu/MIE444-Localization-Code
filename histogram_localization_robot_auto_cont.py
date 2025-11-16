from client_communication import *
import matplotlib.pyplot as plt

from histogram_localization import HistogramLocalization
from robot_control import RobotDrive
from pathfinding import PathfindingRobot


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

RUN_STARTUP_CODE = True
TRIAL_STARTUP = False

if RUN_STARTUP_CODE:
    if TRIAL_STARTUP:
        robot.centering()
    robot.pingSensors()
    plt.subplot(1, 2, 2)
    robot.plotSensorData(plt=plt)
    observed_block_type = block_type_detected(robot.ToFDistancesRaw)
    print(f"Detected block type: {observed_block_type}")
    localizer.update_belief(observed_block_type)
    localizer.visualize_belief(plt, False)

updateHistogram = False
while True:
    # Pathfinding
    pos = localizer.get_most_likely_position()
    current_r, current_c, current_ori = pos
    position_prob = localizer.get_position_probability(current_r, current_c)

    if position_prob > 0.4:
        action, path = pathfinder.get_next_action_to_objective(
            int(current_r),
            int(current_c),
            int(current_ori),
        )
    else:
        action = ""

    if action == "":
        print(
            "Warning: Low confidence in current position estimate. Just move forward."
        )
        if robot.ToFDistances[0] > 150:
            robot.obstacleAvoidanceContinuous()
            updateHistogram = True
        else:
            newFrontend = (robot.currentFrontend + 1) % 4
            robot.changeFrontEnd(newFrontend)
    if action == "forward":
        if robot.currentFrontend == 0:
            if robot.ToFDistances[0] > 150:
                robot.obstacleAvoidanceContinuous()
                updateHistogram = True
            else:
                print("ERROR: Obstacle detected ahead, cannot move forward.")
        else:
            robot.changeFrontEnd(0)
    if action == "right":
        if robot.currentFrontend == 1:
            if robot.ToFDistances[0] > 150:
                robot.obstacleAvoidanceContinuous()
                updateHistogram = True
            else:
                print("ERROR: Obstacle detected ahead, cannot move forward.")
        else:
            robot.changeFrontEnd(1)
    if action == "backward":
        if robot.currentFrontend == 2:
            if robot.ToFDistances[0] > 150:
                robot.obstacleAvoidanceContinuous()
                updateHistogram = True
            else:
                print("ERROR: Obstacle detected ahead, cannot move forward.")
        else:
            robot.changeFrontEnd(2)
    if action == "left":
        if robot.currentFrontend == 3:
            if robot.ToFDistances[0] > 150:
                robot.obstacleAvoidanceContinuous()
                updateHistogram = True
            else:
                print("ERROR: Obstacle detected ahead, cannot move forward.")
        else:
            robot.changeFrontEnd(3)
    if action == "wait":
        robot.sendCommand("h")
    if action == "pickup" or action == "dropoff":
        robot.sendCommand("l")
        with_load = not with_load

    if updateHistogram:
        ############### Histogram Localization Update ##############

        # Movement We just made
        localizer.predict_motion(robot.currentFrontend)

        # Ping Sensors
        robot.pingSensors()
        print(f"Sensor Ping Responses: {robot.ToFDistancesRaw}")
        plt.subplot(1, 2, 2)
        robot.plotSensorData(plt=plt)

        # Detect Block Type
        block_type = block_type_detected(robot.ToFDistances)
        print(f"Block Type Detected = {block_type}")

        # Update Histogram Localization
        localizer.update_belief(block_type)
        plt.subplot(1, 2, 1)
        localizer.visualize_belief(plt, False)
