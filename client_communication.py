import time
from datetime import datetime
import serial
from colorama import Fore


class ClientCommunication:
    def __init__(self, SER: serial.Serial, print_debug: bool = True):
        self.SER = SER
        self.print_debug = print_debug

    # Wrapper functions
    def transmit(self, data):
        """Transmit a command over a serial connection."""
        if self.print_debug:
            print(
                Fore.WHITE
                + f"Transmitting {data} at: {datetime.now().strftime('%H:%M:%S:%f')}"
            )
        self.clear_serial()
        self.SER.write(data.encode("ascii"))

    def receive(self):
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
        if self.print_debug:
            print(
                Fore.WHITE
                + f"Raw response was: {response_raw} at {datetime.now().strftime('%H:%M:%S:%f')}"
            )

        # If response received, return it
        if response_raw:
            return [self.depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
        else:
            return [[False], ""]

    def clear_serial(self, delay_time: float = 0):
        """Wait some time (delay_time) and then clear the serial buffer."""
        if self.SER.in_waiting:
            time.sleep(delay_time)
            if self.print_debug:
                print(
                    Fore.WHITE
                    + f"Clearing Serial... Dumped: {self.SER.read(self.SER.in_waiting)}"
                )

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
        if not any(char in data for char in [FRAMESTART, FRAMEEND, "\n"]):
            return FRAMESTART + data + FRAMEEND + "\n"
        else:
            return False


############## Constant Definitions Begin ##############
### Serial Setup ###
BAUDRATE = 115200  # Baudrate in bps
PORT_SERIAL = "COM8"  # COM port identification
TIMEOUT_SERIAL = 3  # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = "["
FRAMEEND = "]"
CMD_DELIMITER = ","
