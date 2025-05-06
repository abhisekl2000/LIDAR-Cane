import machine
from machine import UART
import time


class TFLuna:

    # def __init__(self, uart_id=1, tx_pin=17, rx_pin=16, baudRate=115200):
    def __init__(self, uart_id, tx_pin, rx_pin, baudRate):
        self.uart = UART(uart_id, baudrate=baudRate, tx=tx_pin, rx=rx_pin)

    def process_lidar_raw_data(self, raw_data):
        frames = []
        i = 0

        while i <= len(raw_data) - 9:
            # Search for the correct frame header (0x59 0x59)
            if raw_data[i] == 0x59 and raw_data[i+1] == 0x59:
                frame = raw_data[i:i+9]

                # Check if the frame length is correct
                if len(frame) == 9:
                    # Extract data from the frame
                    dist = frame[2] | (frame[3] << 8)   #shift frame[3] 8-bits to the left and combine with frame[2] into a single 16-bit integer
                    amp = frame[4] | (frame[5] << 8)
                    temp = frame[6] | (frame[7] << 8)
                    checksum = frame[8]

                    # Compute checksum
                    computed_checksum = sum(frame[:8]) & 0xFF

                    if checksum == computed_checksum:
                        # Convert temperature to Celsius
                        temp_celsius = temp / 8 - 256

                        # Check if distance measurement is reliable
                        if amp < 100 or amp == 65535:
                            dist = None  # unreliable measurement
                            print("Unreliable measurement")

                        frames.append((dist, amp, temp_celsius))
                    else:
                        print("Checksum error in frame:", frame)

                # Move to the next frame
                i += 9
            else:
                i += 1  # Move forward to find the correct start of the frame

        return frames

    def read_lidar(self):
        if self.uart.any():
            raw_data = self.uart.read()
            frames = self.process_lidar_raw_data(raw_data)
            if frames:
                return frames[-1]  # Return the most recent frame
        return None

    def get_distance_strength_temperature(self):
        frame = self.read_lidar()
        if frame:
            # distance_cm, _, _ = frame
            distance_cm, strength, temperature = frame
            if distance_cm is not None:
                distance_m = distance_cm / 100   #cm to m conversion
                return (distance_m, strength, temperature)    
        return None
