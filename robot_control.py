import time
from colorama import Fore

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

        self.lastLoadToFDistances = [8000, 8000]  # top, bottom
        self.LoadToFDistances = [8000, 8000]  # top, bottom

        self.hasPassedCenter = False
        self.pauseInCenter = True
        self.previousOffset = 0

        self.autoChangeFrontEnd = False

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
            print(Fore.BLUE + f"Sensor Responses at {time_rx}: {responses}")
            # Check validity of responses
            if len(responses) != (5 if raw_cmd == "p" else 9):
                if self.verboseConsole:
                    print(Fore.RED + "Invalid sensor response length, trying again.")
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

    def sendCommand(self, raw_cmd, timeout=0):
        if timeout == 0:
            timeout = self.RESPONSE_TIMEOUT
        packet_tx = self.packetize(raw_cmd)
        if packet_tx:
            self.transmit(packet_tx)
        start_time = time.time()
        while time.time() - start_time < timeout:
            [responses, time_rx] = self.receive()
            if responses[0] == raw_cmd:
                continue
            if responses[0] == "+":
                break
        if self.verboseConsole:
            print(Fore.GREEN + f"Command Response at {time_rx}: {responses}")
        return responses

    def changeFrontEnd(self, new_frontend, center_after_change=False):
        # If new_frontend is the same as current, do nothing
        if new_frontend == self.currentFrontend:
            if self.verboseConsole:
                print(
                    Fore.YELLOW
                    + "New frontend is the same as current, no change needed."
                )
            return
        # If new_frontend is opposite to current, no centering checks needed
        if (new_frontend - self.currentFrontend) % 4 == 2:
            if self.verboseConsole:
                print(
                    Fore.YELLOW
                    + "New frontend is opposite to current, no centering checks needed."
                )
            self.sendCommand("r" + str(new_frontend))
            if center_after_change:
                self.performBlockCentering()
            return
        # For adjacent frontends, perform centering checks
        if self.verboseConsole:
            print(Fore.YELLOW + "Changing frontend with centering checks.")
        centered, _ = self.centreinblock()
        if centered:
            if self.verboseConsole:
                print(
                    Fore.YELLOW + "Robot is well centered, changing frontend directly."
                )
            self.sendCommand("r" + str(new_frontend))
            if center_after_change:
                self.performBlockCentering()
            return
        else:
            if self.verboseConsole:
                print(
                    Fore.YELLOW
                    + "Robot is offcentered, adjusting position before changing frontend."
                )
            self.performBlockCentering()
            self.sendCommand("r" + str(new_frontend))
            if center_after_change:
                self.performBlockCentering()
            return

    def avoidFrontWall(self):
        # Obstacle detected in front, stop and back up
        if self.verboseConsole:
            print(Fore.MAGENTA + "Obstacle detected in front, backing up and rotating.")
        self.sendCommand("s100")
        if self.OMNIWHEELDRIVE and self.autoChangeFrontEnd:
            if self.ToFDistances[1] < 150:
                new_front = (self.currentFrontend + 3) % 4
                self.sendCommand(f"r{new_front}")
            else:
                new_front = (self.currentFrontend + 1) % 4
                self.sendCommand(f"r{new_front}")

    def avoidSideWalls(self):
        # Against the wall is 30 mm
        if self.ToFDistances[1] < 50:
            # Avoiding right wall collisions
            if self.verboseConsole:
                print(
                    Fore.MAGENTA
                    + f"Obstacle too close on the right sensor: {self.ToFDistances[1]}mm, veering left."
                )
            movement_duration = (
                75 - self.ToFDistances[1]
            ) * 5  # Adjust duration based on distance
            self.sendCommand(f"a{movement_duration}")
            time.sleep(movement_duration / 1000)
            movement_duration /= 2
            self.sendCommand(f"q{movement_duration}")
            time.sleep(movement_duration / 1000)
            return True

        elif self.ToFDistances[3] < 50:
            # Avoiding left wall collisions
            if self.verboseConsole:
                print(
                    Fore.MAGENTA
                    + f"Obstacle too close on the left sensor: {self.ToFDistances[3]}mm, veering right."
                )
            movement_duration = (
                75 - self.ToFDistances[3]
            ) * 5  # Adjust duration based on distance
            self.sendCommand(f"d{movement_duration}")
            time.sleep(movement_duration / 1000)
            movement_duration /= 2
            self.sendCommand(f"e{movement_duration}")
            time.sleep(movement_duration / 1000)
            return True
        return False

    def hugSideWalls(self, rotation=False):
        if (
            self.ToFDistances[3] > 90
            and self.ToFDistances[3] < 180
            and self.ToFDistances[1] >= 150
        ):
            if self.verboseConsole:
                print(
                    Fore.MAGENTA
                    + f"Too far from left sensor: {self.ToFDistances[3]}mm, veering left."
                )
            movement_duration = max(
                (self.ToFDistances[3] - 75) * 5, 100
            )  # Adjust duration based on distance
            self.sendCommand(f"a{movement_duration}")
            time.sleep(movement_duration / 1000)
            if rotation:
                movement_duration /= 4
                self.sendCommand(f"q{movement_duration}")
                time.sleep(movement_duration / 1000)

        elif (
            self.ToFDistances[1] > 90
            and self.ToFDistances[1] < 180
            and self.ToFDistances[3] >= 150
        ):
            if self.verboseConsole:
                print(
                    Fore.MAGENTA
                    + f"Too far from right sensor: {self.ToFDistances[1]}mm, veering right."
                )
            movement_duration = max(
                (self.ToFDistances[1] - 75) * 5, 100
            )  # Adjust duration based on distance
            self.sendCommand(f"d{movement_duration}")
            time.sleep(movement_duration / 1000 + 0.5)
            if rotation:
                movement_duration /= 4
                self.sendCommand(f"e{movement_duration}")
                time.sleep(movement_duration / 1000 + 0.5)

    def obstacleAvoidance(self, ping=True, duration=500):
        if self.verboseConsole:
            print(Fore.CYAN + "Starting obstacle avoidance routine...")
        if ping:
            self.pingSensors()

        if self.ToFDistances[0] < 60:
            self.avoidFrontWall()

        if not self.avoidSideWalls():
            self.hugSideWalls()

        if self.verboseConsole:
            print(Fore.CYAN + "Path clear, moving forward.")
        self.sendCommand(f"f{duration}")

    def obstacleAvoidanceContinuous(self, ping=True, duration=200):
        self.hasPassedCenter = False
        while not (self.hasPassedCenter and self.pauseInCenter):
            if self.verboseConsole:
                print(Fore.CYAN + "Starting obstacle avoidance routine...")

            if ping:
                self.pingSensors()

            if self.ToFDistances[0] < 90:
                self.avoidFrontWall()

            if not self.avoidSideWalls():
                self.hugSideWalls()

            if self.verboseConsole:
                print(Fore.CYAN + "Path clear, moving forward.")
            self.sendCommand(f"f{duration}")
            # time.sleep(0.1)
            # self.checkCentering()
            self.centreinblock()
        self.sendCommand("h")
        self.performBlockCentering()

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
                print(
                    Fore.GREEN
                    + f"M1: {lastMeasure1} M2: {lastMeasure2} M3: {lastMeasure3}"
                )
            if self.ToFDistances[0] > 300 and self.ToFDistances[1] < 150:
                if lastMeasure1 >= lastMeasure2 and lastMeasure3 >= lastMeasure2:
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
                print(
                    Fore.GREEN
                    + "No walls detected within 150mm, skipping parallelization."
                )
            return

        # find sensor with the closest wall
        min_distance = min(self.ToFDistances)
        min_index = self.ToFDistances.index(min_distance)

        # Start rotating CW, if distance starts decreases, continue rotating CW
        # until distance starts increasing, then do a small CCW rotation and stop.
        # Otherwise if distance starts increases, stop, and start rotating CCW
        # until distance starts increasing, then do a small CW rotation and stop.
        self.sendCommand("e100")  # small CW rotation
        time.sleep(0.3)
        self.pingSensors()
        distance_plus_CW = self.ToFDistances[min_index]
        if distance_plus_CW <= min_distance:
            # continue rotating CW
            if self.verboseConsole:
                print(Fore.GREEN + f"Continuing CW rotation for parallelization. {distance_plus_CW} <= {min_distance}")
            while True:
                self.sendCommand("e100")  # continue CW rotation
                time.sleep(0.3)
                self.pingSensors()
                new_distance = self.ToFDistances[min_index]
                if new_distance > distance_plus_CW:
                    # distance started increasing, do small CCW rotation and stop
                    if self.verboseConsole:
                        print(
                            Fore.GREEN
                            + f"Distance increased: {new_distance}, adjusting with small CCW rotation."
                        )
                    self.sendCommand("q100")  # small CCW rotation
                    time.sleep(0.3)
                    break
                print(Fore.GREEN + f"Distance decreased: {new_distance}.")
                distance_plus_CW = new_distance
        else:
            min_distance = (
                distance_plus_CW  # to avoid early stopping in reverse rotation
            )
            # start rotating CCW
            if self.verboseConsole:
                print(Fore.GREEN + f"Starting CCW rotation for parallelization. {distance_plus_CW} > {min_distance}")
            self.sendCommand("q100")  # small CCW rotation (back to original + CCW)
            time.sleep(0.3)
            self.pingSensors()
            distance_plus_CCW = self.ToFDistances[min_index]
            while True:
                if distance_plus_CCW <= min_distance:
                    # continue rotating CCW
                    self.sendCommand("q100")  # continue CCW rotation
                    time.sleep(0.3)
                    self.pingSensors()
                    new_distance = self.ToFDistances[min_index]
                    if new_distance > distance_plus_CCW:
                        # distance started increasing, do small CW rotation and stop
                        if self.verboseConsole:
                            print(
                                Fore.GREEN
                                + f"Distance increased: {new_distance}, adjusting with small CW rotation."
                            )
                        self.sendCommand("e100")  # small CW rotation
                        time.sleep(0.3)
                        break
                    print(Fore.GREEN + f"Distance decreased: {new_distance}.")
                    distance_plus_CCW = new_distance
                else:
                    break

    def performBlockCentering(self):
        centered = False
        while not centered:
            [centered, offcenter] = self.centreinblock()
            if not centered:
                if offcenter > 0:
                    if self.verboseConsole:
                        print(
                            Fore.MAGENTA
                            + f"Robot is before the center of the block, moving forward by {offcenter}mm."
                        )
                    duration = min(offcenter * 8, 1000)
                    self.sendCommand(f"f{duration}")
                    time.sleep(duration / 1000 + 0.5)
                else:
                    if self.verboseConsole:
                        print(
                            Fore.MAGENTA
                            + f"Robot is beyond the center of the block, moving backward by {abs(offcenter)}mm."
                        )
                    duration = min(abs(offcenter) * 8, 1000)
                    self.sendCommand(f"s{duration}")
                    time.sleep(duration / 1000 + 0.5)

    def centreinblock(self, ping=True):
        if ping:
            self.pingSensors()
        blocklength = 305  # Length of a block
        tolerance = 25  # tolerance around center
        middle = 76  # reading from sensor when centered
        mult = 1  # used to correct signed direction of travel to center
        correction = 0
        minDis = min(
            self.ToFDistances[0] + middle, self.ToFDistances[2] + middle
        )  # Shortest length between front and back wall corrected to robot center
        if minDis // 305 == 1:
            correction = 20
        elif minDis // 305 > 1:
            correction = 40
        minDis = minDis - correction
        if (
            self.ToFDistances[0] < self.ToFDistances[2]
        ):  # Sets mult based on larger front or back length (1 forward, -1 backwards)
            mult = -1
        print(Fore.GREEN + "Robot distance wall:" + str(minDis))
        minDis = (
            minDis % blocklength
        )  # distance from biginning of current block to robot center
        print(Fore.GREEN + "Robot distance to start of block:" + str(minDis))
        middleDis = 2 * middle - minDis  # distance from robot center to block center
        print(Fore.GREEN + "Distance to center of block:" + str(middleDis * mult))
        middleDis *= mult

        if middleDis < 25 and self.previousOffset > 25:
            self.hasPassedCenter = True
        self.previousOffset = middleDis

        if (
            -tolerance < middleDis < tolerance
        ):  # if block center distance is whithin tolerance, robot is in center
            print(Fore.GREEN + "Robot is centered in block!!!")
            return [True, middleDis]
        else:
            self.pauseInCenter = True
            return [False, middleDis]

    def pingLoadSensors(self, try_again=True):
        # CALIBRATION_OFFSET = 200
        raw_cmd = "v"
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
            print(Fore.BLUE + f"Sensor Responses at {time_rx}: {responses}")
            # Check validity of responses
            if len(responses) != 2:
                if self.verboseConsole:
                    print(Fore.RED + "Invalid sensor response length, trying again.")
                    if try_again:
                        self.pingSensors(raw_cmd, try_again=False)
                return
        self.lastLoadToFDistances = self.LoadToFDistances.copy()
        self.LoadToFDistances[0] = int(responses[0])
        self.LoadToFDistances[1] = int(responses[1])

    def detectLoad(self):
        # self.sendCommand("z")
        # self.sendCommand("z")
        # self.sendCommand("z")
        self.sendCommand("r0")
        TOLERANCE = 70  # mm
        TURN_DURATION = 50  # ms
        SENSOR_LIMIT = 500 # mm
        self.pingLoadSensors()
        lockedOnLoad = False
        while (
            abs(self.LoadToFDistances[0] - self.LoadToFDistances[1]) < TOLERANCE
            or self.LoadToFDistances[1] > 130
            or (self.LoadToFDistances[0] > SENSOR_LIMIT and self.LoadToFDistances[1] > SENSOR_LIMIT)
        ):
            if (
                abs(self.LoadToFDistances[0] - self.LoadToFDistances[1]) > TOLERANCE
                and not lockedOnLoad
            ):
                # Found load, trigger lock on
                print(Fore.MAGENTA + f"Load detected at {self.LoadToFDistances}, locking on.")
                lockedOnLoad = True
            elif (
                abs(self.LoadToFDistances[0] - self.LoadToFDistances[1]) < TOLERANCE
                and lockedOnLoad
            ):
                # Lost Load, hard turn CW and sweep CCW
                print(Fore.MAGENTA + f"Lost load at {self.LoadToFDistances}, sweeping for load.")
                lockedOnLoad = False
                self.sendCommand("e200")
                time.sleep(0.5)
            if lockedOnLoad:
                # Inch Forward to Load
                print(Fore.MAGENTA + f"Approaching load at {self.LoadToFDistances}.")
                self.sendCommand("f50")
                time.sleep(0.1)
                self.sendCommand("d70")
                time.sleep(0.1)
                self.pingLoadSensors()
            else:
                # Sweep for Load
                print(Fore.MAGENTA + f"Sweeping for load at {self.LoadToFDistances}. Diff: {self.LoadToFDistances[0] - self.LoadToFDistances[1]}")
                self.sendCommand(f"q{TURN_DURATION}")
                time.sleep(0.2)
                self.pingLoadSensors()
                # EXPERIMENTAL CHECKING FOR CHANGE DIFF
                # topDiff = self.lastLoadToFDistances[0] - self.LoadToFDistances[0]
                # bottomDiff = self.lastLoadToFDistances[1] - self.LoadToFDistances[1]
                # print(Fore.MAGENTA +
                #     f"Top: {self.LoadToFDistances[0]}  Bottom: {self.LoadToFDistances[1]}"
                # )
                # print(Fore.MAGENTA + f"Top Diff: {topDiff}  Bottom Diff: {bottomDiff}")

        print(Fore.MAGENTA + f"Load in range {self.LoadToFDistances}.")
        # Gripper open and down
        servo0, servo0_up, servo0_down = 0, 120, 10
        servo1, servo1_open, servo1_close = 1, 0, 110

        self.sendCommand(f"l{servo1}{servo1_open}")
        time.sleep(0.5)
        self.sendCommand(f"l{servo0}{servo0_down}")
        time.sleep(0.5)

        # Inch forward to block and align block
        for _ in range(10):
            self.sendCommand("f50")
            time.sleep(0.1)
            self.sendCommand("d50")
            time.sleep(0.1)

        # Gripper close and up
        self.sendCommand(f"l{servo1}{servo1_close}")
        time.sleep(0.5)
        self.sendCommand(f"l{servo0}{servo0_up}")
        time.sleep(0.5)

        self.sendCommand("x")
        self.sendCommand("x")
        self.sendCommand("x")

    def dropLoad(self):
        # GRIPPER PROCEDURE
        servo0, servo0_up, servo0_down = 0, 120, 10
        servo1, servo1_open = 1, 0

        self.sendCommand(f"l{servo0}{servo0_down}")
        time.sleep(0.5)
        self.sendCommand(f"l{servo1}{servo1_open}")
        time.sleep(0.5)
        self.sendCommand(f"l{servo0}{servo0_up}")
        time.sleep(0.5)
