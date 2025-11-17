import time
from datetime import datetime
import serial


class ClientCommunication:
    def __init__(self, SER: serial.Serial):
        self.SER = SER

    # Wrapper functions
    def transmit(self, data):
        """Selects whether to use serial or tcp for transmitting."""
        print(f"Transmitting at: {datetime.now().strftime('%H:%M:%S:%f')}")
        self.transmit_serial(data)

    def receive(self):
        """Selects whether to use serial or tcp for receiving."""
        return self.receive_serial()

    # Serial communication functions
    def transmit_serial(self, data):
        """Transmit a command over a serial connection."""
        self.clear_serial()
        self.SER.write(data.encode("ascii"))

    def receive_serial(self):
        """Receive a reply over a serial connection."""
        self.SER.timeout = 0.1  # Short timeout to start reading
        start_time = time.time()
        response_raw = ""
        while time.time() < start_time + TIMEOUT_SERIAL:
            response_char = self.SER.read()
            response_char = response_char.decode("ascii")
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

        print(f"Raw response was: {response_raw} at {datetime.now().strftime('%H:%M:%S:%f')}")

        # If response received, return it
        if response_raw:
            return [self.depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
        else:
            return [False]

    def clear_serial(self, delay_time: float = 0):
        """Wait some time (delay_time) and then clear the serial buffer."""
        if self.SER.in_waiting:
            time.sleep(delay_time)
            print(f"Clearing Serial... Dumped: {self.SER.read(self.SER.in_waiting)}")

    # Packetization and validation functions
    def depacketize(self, data_raw: str):
        """
        Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
        """

        # Locate start and end framing characters
        start = data_raw.find(FRAMESTART)
        end = data_raw.find(FRAMEEND)

        # Check that the start and end framing characters are present, then return commands as a list
        if start >= 0 and end >= start:
            data = (
                data_raw[start + 1 : end]
                .replace(f"{FRAMEEND}{FRAMESTART}", ",")
                .split(",")
            )
            return data
        else:
            return [False]

    def packetize(self, data: str):
        """
        Take a message that is to be sent to the command script and packetize it with start and end framing.
        """

        # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
        forbidden = [FRAMESTART, FRAMEEND, "\n"]
        check_fail = any(char in data for char in forbidden)

        if not check_fail:
            return FRAMESTART + data + FRAMEEND + "\n"

        return False


############## Constant Definitions Begin ##############
### Serial Setup ###
BAUDRATE = 9600  # Baudrate in bps
PORT_SERIAL = "COM8"  # COM port identification
TIMEOUT_SERIAL = 3  # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = "["
FRAMEEND = "]"
CMD_DELIMITER = ","
