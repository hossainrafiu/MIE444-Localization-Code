from localization_robot import packetize, transmit, receive
import time


class ObstacleAvoidance:
    def __init__(self):
        self.verboseConsole = True
        self.RESPONSE_TIMEOUT = 3  # seconds
        self.MOVELEFTWHENPOSSIBLE = False
        self.MOVERIGHTWHENPOSSIBLE = False
        self.OMNIWHEELDRIVE = True

        self.lastUSDistances = [0, 0, 0, 0, 0, 0, 0, 0]
        self.USDistances = [0, 0, 0, 0, 0, 0, 0, 0]
        self.lastToFDistances = [8000, 8000, 8000, 8000]  # front, right, back, left
        self.ToFDistances = [8000, 8000, 8000, 8000]  # front, right, back, left
        self.currentFrontend = 0

    def pingSensors(self, raw_cmd="p"):
        packet_tx = packetize(raw_cmd)
        if packet_tx:
            transmit(packet_tx)
        start_time = time.time()
        while time.time() - start_time < self.RESPONSE_TIMEOUT:
            [responses, time_rx] = receive()
            if responses[0] == raw_cmd:
                continue
            if responses[0] is not False:
                break
        if self.verboseConsole:
            print(f"Sensor Responses at {time_rx}: {responses}")
        self.currentFrontend = int(responses[-1])
        self.lastToFDistances = self.ToFDistances.copy()
        # shift responses to match front direction
        if raw_cmd == "p":
            for i in range(4):
                sensor_index = (i - self.currentFrontend) % 4
                self.ToFDistances[sensor_index] = int(responses[i])
        if raw_cmd == "u":
            for i in range(8):
                sensor_index = (i - self.currentFrontend) % 8
                self.USDistances[sensor_index] = int(responses[i])

    def sendCommand(self, raw_cmd):
        packet_tx = packetize(raw_cmd)
        if packet_tx:
            transmit(packet_tx)
        while True:
            [responses, time_rx] = receive()
            if responses[0] == raw_cmd:
                continue
            if responses[0] == "+":
                break
        if self.verboseConsole:
            print(f"Command Response at {time_rx}: {responses}")
        return responses

    def moveLeftWhenPossible(self):
        # try to move left if possible
        if self.ToFDistances[3] > 150:
            if self.verboseConsole:
                print(
                    "Path clear on the left, carefully moving straight and then rotating."
                )
            self.sendCommand("f500")
            if self.OMNIWHEELDRIVE:
                new_front = (self.currentFrontend + 3) % 4
                self.sendCommand(f"r{new_front}")
            else:
                self.sendCommand("r900")
            self.MOVELEFTWHENPOSSIBLE = False

    def moveRightWhenPossible(self):
        # try to move right if possible
        if self.ToFDistances[1] > 150:
            if self.verboseConsole:
                print(
                    "Path clear on the right, carefully moving straight and then rotating."
                )
            self.sendCommand("f500")
            if self.OMNIWHEELDRIVE:
                new_front = (self.currentFrontend + 1) % 4
                self.sendCommand(f"r{new_front}")
            else:
                self.sendCommand("r-900")
            self.MOVERIGHTWHENPOSSIBLE = False

    def avoidCornersIfTurning(self):
        if (
            self.lastToFDistances[1] < 200
            and abs(self.lastToFDistances[1] - self.ToFDistances[1]) > 100
        ):
            if self.verboseConsole:
                print("Significant change in right sensor distance.")
            # moving forward to avoid wall collision if wanting to move right
            self.sendCommand("f400")

        elif (
            self.lastToFDistances[3] < 200
            and abs(self.lastToFDistances[3] - self.ToFDistances[3]) > 100
        ):
            if self.verboseConsole:
                print("Significant change in left sensor distance.")
            # moving forward to avoid wall collision if wanting to move left
            self.sendCommand("f400")

    def avoidFrontWall(self):
        # Obstacle detected in front, stop and back up
        if self.verboseConsole:
            print("Obstacle detected in front, backing up and rotating.")
        self.sendCommand("b100")
        if self.OMNIWHEELDRIVE:
            if self.ToFDistances[1] < 150:
                new_front = (self.currentFrontend + 3) % 4
                self.sendCommand(f"r{new_front}")
            else:
                new_front = (self.currentFrontend + 1) % 4
                self.sendCommand(f"r{new_front}")
        else:
            if self.ToFDistances[1] < 150:
                # Obstacle on the right, rotate ACW
                if self.verboseConsole:
                    print("Obstacle detected on the right, rotating counter-clockwise.")
                self.sendCommand("r900")
            else:
                # No obstacle on the right, rotate CW
                if self.verboseConsole:
                    print("No obstacle on the right, rotating clockwise.")
                self.sendCommand("r-900")

    def avoidSideWalls(self):
        if self.ToFDistances[1] < 60:
            # Avoiding right wall collisions
            if self.verboseConsole:
                print(
                    f"Obstacle too close on the right sensor: {self.ToFDistances[1]}mm, veering left."
                )
            movement_duration = max(
                60 - self.ToFDistances[1] * 10, 100
            )  # Adjust duration based on distance
            self.sendCommand(f"l{movement_duration}")
            return True

        elif self.ToFDistances[3] < 60:
            # Avoiding left wall collisions
            if self.verboseConsole:
                print(
                    f"Obstacle too close on the left sensor: {self.ToFDistances[3]}mm, veering right."
                )
            movement_duration = max(
                60 - self.ToFDistances[3] * 10, 100
            )  # Adjust duration based on distance
            self.sendCommand(f"r{movement_duration}")
            return True
        return False

    def hugSideWalls(self):
        if (
            self.ToFDistances[3] > 90
            and self.ToFDistances[3] < 150
            and self.ToFDistances[1] >= 150
        ):
            if self.verboseConsole:
                print(
                    f"Too far from left sensor: {self.ToFDistances[3]}mm, veering left."
                )
            movement_duration = max(
                self.ToFDistances[3] - 90, 100
            )  # Adjust duration based on distance
            self.sendCommand(f"l{movement_duration}")

        elif (
            self.ToFDistances[1] > 90
            and self.ToFDistances[1] < 150
            and self.ToFDistances[3] >= 150
        ):
            if self.verboseConsole:
                print(
                    f"Too far from right sensor: {self.ToFDistances[1]}mm, veering right."
                )
            movement_duration = max(
                self.ToFDistances[1] - 90, 100
            )  # Adjust duration based on distance
            self.sendCommand(f"r{movement_duration}")

    def obstacleAvoidance(self):
        if self.verboseConsole:
            print("Starting obstacle avoidance routine...")
        self.pingSensors()

        if self.MOVELEFTWHENPOSSIBLE:
            self.moveLeftWhenPossible()

        if self.MOVERIGHTWHENPOSSIBLE:
            self.moveRightWhenPossible()

        self.avoidCornersIfTurning()

        if self.ToFDistances[0] < 60:
            self.avoidFrontWall()

        if not self.avoidSideWalls():
            self.hugSideWalls()

        if self.verboseConsole:
            print("Path clear, moving forward.")
        self.sendCommand("f500")
        time.sleep(0.5)
