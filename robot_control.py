import time

RECURSIVE_ALLIGN = False


class RobotDrive:
    def __init__(self, packetize, transmit, receive):
        self.verboseConsole = True
        self.RESPONSE_TIMEOUT = 6  # seconds
        self.MOVELEFTWHENPOSSIBLE = False
        self.MOVERIGHTWHENPOSSIBLE = False
        self.OMNIWHEELDRIVE = True
        self.packetize = packetize
        self.transmit = transmit
        self.receive = receive

        self.lastUSDistances = [0, 0, 0, 0, 0, 0, 0, 0]
        self.USDistances = [0, 0, 0, 0, 0, 0, 0, 0]
        self.USDistancesRaw = [0, 0, 0, 0, 0, 0, 0, 0]
        self.lastToFDistances = [8000, 8000, 8000, 8000]  # front, right, back, left
        self.ToFDistances = [8000, 8000, 8000, 8000]  # front, right, back, left
        self.ToFDistancesRaw = [8000, 8000, 8000, 8000]
        self.currentFrontend = 0

        self.centeringThreshold = 50  # mm

    def pingSensors(self, raw_cmd="p", try_again=True):
        packet_tx = self.packetize(raw_cmd)
        if packet_tx:
            self.transmit(packet_tx)
        start_time = time.time()
        while time.time() - start_time < self.RESPONSE_TIMEOUT:
            [responses, time_rx] = self.receive()
            if responses[0] == raw_cmd:
                continue
            if responses[0] != "+" and responses[0] is not False:
                break
        if self.verboseConsole:
            print(f"Sensor Responses at {time_rx}: {responses}")
            # Check validity of responses
            if len(responses) != (5 if raw_cmd == "p" else 9):
                if self.verboseConsole:
                    print("Invalid sensor response length, trying again.")
                    if try_again:
                        self.pingSensors(raw_cmd, try_again=False)
                return
        self.currentFrontend = int(responses[-1])
        # shift responses to match front direction
        if raw_cmd.find("p") != -1:
            self.lastToFDistances = self.ToFDistances.copy()
            for i in range(4):
                sensor_index = (i - self.currentFrontend) % 4
                self.ToFDistancesRaw[i] = int(responses[i])
                self.ToFDistances[sensor_index] = int(responses[i])
        elif raw_cmd.find("u") != -1:
            self.lastUSDistances = self.USDistances.copy()
            for i in range(8):
                sensor_index = (i - self.currentFrontend) % 8
                self.USDistancesRaw[i] = int(responses[i])
                self.USDistances[sensor_index] = int(responses[i])

    def sendCommand(self, raw_cmd):
        packet_tx = self.packetize(raw_cmd)
        if packet_tx:
            self.transmit(packet_tx)
        start_time = time.time()
        while time.time() - start_time < self.RESPONSE_TIMEOUT:
            [responses, time_rx] = self.receive()
            if responses[0] == raw_cmd:
                continue
            if responses[0] == "+":
                break
        if self.verboseConsole:
            print(f"Command Response at {time_rx}: {responses}")
        return responses

    def changeFrontEnd(self, new_frontend):
        # If new_frontend is the same as current, do nothing
        if new_frontend == self.currentFrontend:
            if self.verboseConsole:
                print("New frontend is the same as current, no change needed.")
            return
        # If new_frontend is opposite to current, no centering checks needed
        if (new_frontend - self.currentFrontend) % 4 == 2:
            if self.verboseConsole:
                print(
                    "New frontend is opposite to current, no centering checks needed."
                )
            self.sendCommand("r" + str(new_frontend))
            return
        # For adjacent frontends, perform centering checks
        if self.verboseConsole:
            print("Changing frontend with centering checks.")
        offcenter = self.checkCentering()
        if abs(offcenter) < self.centeringThreshold:
            if self.verboseConsole:
                print("Robot is well centered, changing frontend directly.")
            self.sendCommand("r" + str(new_frontend))
            return
        elif offcenter >= self.centeringThreshold:
            if self.verboseConsole:
                print(
                    "Robot is in behind of the center of the block, move forward before turning."
                )
            duration = min(offcenter * 5, 1000)
            self.sendCommand(f"f{duration}")
            time.sleep(duration)
            self.sendCommand("r" + str(new_frontend))
            return
        return

    def avoidFrontWall(self):
        # Obstacle detected in front, stop and back up
        if self.verboseConsole:
            print("Obstacle detected in front, backing up and rotating.")
        self.sendCommand("s100")
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
                self.sendCommand("q900")
            else:
                # No obstacle on the right, rotate CW
                if self.verboseConsole:
                    print("No obstacle on the right, rotating clockwise.")
                self.sendCommand("e900")

    def avoidSideWalls(self):
        if self.ToFDistances[1] < 60:
            # Avoiding right wall collisions
            if self.verboseConsole:
                print(
                    f"Obstacle too close on the right sensor: {self.ToFDistances[1]}mm, veering left."
                )
            movement_duration = max(
                60 - self.ToFDistances[1] * 10, 200
            )  # Adjust duration based on distance
            self.sendCommand(f"a{movement_duration}")
            return True

        elif self.ToFDistances[3] < 60:
            # Avoiding left wall collisions
            if self.verboseConsole:
                print(
                    f"Obstacle too close on the left sensor: {self.ToFDistances[3]}mm, veering right."
                )
            movement_duration = max(
                60 - self.ToFDistances[3] * 10, 200
            )  # Adjust duration based on distance
            self.sendCommand(f"d{movement_duration}")
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
            self.sendCommand(f"a{movement_duration}")

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
            self.sendCommand(f"d{movement_duration}")

    def obstacleAvoidance(self, ping=True, duration=500):
        if self.verboseConsole:
            print("Starting obstacle avoidance routine...")
        if ping:
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
        self.sendCommand(f"f{duration}")

    def obstacleAvoidanceContinuous(self, ping=True, duration=30000):
        while not self.inCenterOfNextBlock:
            if self.verboseConsole:
                print("Starting obstacle avoidance routine...")

            if ping:
                self.pingSensors()

            # self.avoidCornersIfTurning()

            if self.ToFDistances[0] < 60:
                self.avoidFrontWall()

            if not self.avoidSideWalls():
                self.hugSideWalls()

            if self.verboseConsole:
                print("Path clear, moving forward.")
            self.sendCommand(f"f{duration}")
            time.sleep(0.1)
            self.checkCentering()

    def plotSensorData(self, plt):
        sensors = []
        if self.ToFDistancesRaw[0] < 1500:
            sensors.append((0, self.ToFDistancesRaw[0] + 75))
        if self.ToFDistancesRaw[2] < 1500:
            sensors.append((0, -self.ToFDistancesRaw[2] - 75))
        if self.ToFDistancesRaw[3] < 1500:
            sensors.append((-self.ToFDistancesRaw[3] - 75, 0))
        if self.ToFDistancesRaw[1] < 1500:
            sensors.append((self.ToFDistancesRaw[1] + 75, 0))
        USsensors = []
        if self.USDistancesRaw[0] > 0:
            USsensors.append((-self.USDistancesRaw[0], 75))
        if self.USDistancesRaw[1] > 0:
            USsensors.append((self.USDistancesRaw[1], 75))
        if self.USDistancesRaw[2] > 0:
            USsensors.append((75, self.USDistancesRaw[2]))
        if self.USDistancesRaw[3] > 0:
            USsensors.append((75, -self.USDistancesRaw[3]))
        if self.USDistancesRaw[4] > 0:
            USsensors.append((self.USDistancesRaw[4], -75))
        if self.USDistancesRaw[5] > 0:
            USsensors.append((-self.USDistancesRaw[5], -75))
        if self.USDistancesRaw[6] > 0:
            USsensors.append((-75, -self.USDistancesRaw[6]))
        if self.USDistancesRaw[7] > 0:
            USsensors.append((-75, self.USDistancesRaw[7]))
        plt.scatter(
            [s[0] for s in sensors],
            [s[1] for s in sensors],
            color="blue",
            label="ToF Sensors",
        )
        plt.scatter(
            [s[0] for s in USsensors],
            [s[1] for s in USsensors],
            color="red",
            label="Ultrasonic Sensors",
        )
        # Plot circle for robot body
        circle = plt.Circle((0, 0), 75, color="gray", fill=False, label="Robot Body")
        plt.gca().add_artist(circle)
        plt.scatter(0, 0, color="green", label="Robot Position")
        # Plot multiple Squares for likely wall locations
        for i in range(5):
            square = plt.Rectangle(
                (-150 - 300 * i, -150 - 300 * i),
                300 * (i * 2 + 1),
                300 * (i * 2 + 1),
                color="lightgray",
                fill=False,
            )
            plt.gca().add_artist(square)
        # Plot arrow for front direction
        if self.currentFrontend == 0:
            plt.arrow(
                0, 0, 0, 100, head_width=10, head_length=15, fc="green", ec="green"
            )
        if self.currentFrontend == 1:
            plt.arrow(
                0, 0, 100, 0, head_width=10, head_length=15, fc="green", ec="green"
            )
        if self.currentFrontend == 2:
            plt.arrow(
                0, 0, 0, -100, head_width=10, head_length=15, fc="green", ec="green"
            )
        if self.currentFrontend == 3:
            plt.arrow(
                0, 0, -100, 0, head_width=10, head_length=15, fc="green", ec="green"
            )
        plt.title("Sensor Readings Visualization")
        plt.legend()
        # Equal scaling for x and y axes
        plt.axis("equal")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.grid(True)
        plt.show(block=False)

    def centering(self):
        lastMeasure1 = 8000
        lastMeasure2 = 8000
        lastMeasure3 = 8000
        for _ in range(40):
            self.sendCommand("h")  # halt
            self.sendCommand("e200")  # rotate CW
            time.sleep(0.5)
            self.sendCommand("h")  # halt
            self.pingSensors()
            lastMeasure3 = lastMeasure2
            lastMeasure2 = lastMeasure1
            lastMeasure1 = self.ToFDistances[1]
            if self.verboseConsole:
                print(f"M1: {lastMeasure1} M2: {lastMeasure2} M3: {lastMeasure3}")
            if self.ToFDistances[0] > 300 and self.ToFDistances[1] < 150:
                if lastMeasure1 > lastMeasure2 and lastMeasure3 > lastMeasure2:
                    self.sendCommand("q200")  # rotate CCW
                    time.sleep(0.5)
                    self.sendCommand("h")  # halt
                    return

    def simpleParallelize(self, ping=True):
        if ping:
            self.pingSensors()
        # check if no sensor detects a wall within 150mm, then return
        if (
            self.ToFDistances[0] > 150
            and self.ToFDistances[1] > 150
            and self.ToFDistances[2] > 150
            and self.ToFDistances[3] > 150
        ):
            if self.verboseConsole:
                print("No walls detected within 150mm, skipping parallelization.")
            return
        # find sensor with the closest wall
        min_distance = min(self.ToFDistances)
        min_index = self.ToFDistances.index(min_distance)

        # align with that wall, do small rotations in both directions to find the best alignment
        min_distance_plus_CW = 8000
        min_distance_plus_CCW = 8000
        self.sendCommand("e100")  # small CW rotation
        time.sleep(0.3)
        self.pingSensors()
        min_distance_plus_CW = self.ToFDistances[min_index]
        self.sendCommand("q200")  # small CCW rotation (back to original + CCW)
        time.sleep(0.3)
        self.pingSensors()
        min_distance_plus_CCW = self.ToFDistances[min_index]
        # decide which direction was better compared to original distance
        if (
            min_distance_plus_CW < min_distance
            and min_distance_plus_CW < min_distance_plus_CCW
        ):
            if self.verboseConsole:
                print("Adjusting alignment: rotating CW.")
            self.sendCommand("e200")  # rotate CW
            time.sleep(0.3)
        elif (
            min_distance_plus_CCW < min_distance
            and min_distance_plus_CCW < min_distance_plus_CW
        ):
            if self.verboseConsole:
                print("Adjusting alignment: rotating CCW.")
            self.sendCommand("q100")  # rotate CCW
            time.sleep(0.3)
        else:
            if self.verboseConsole:
                print("Originally well aligned, no adjustment needed.")
            self.sendCommand("e100")  # rotate back to original position

    # Need to implement
    def checkCentering(self):
        return
