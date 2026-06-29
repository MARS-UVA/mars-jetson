import struct
from collections import deque
from serial_node.feedback_mappings import START_BYTE, FEEDBACK_PACKETS
from rclpy.logging import get_logger

class Serial:
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.logger = get_logger('mock_serial')
        self.buffer = deque()  # Use deque for efficient pops from the left
        self.reset_input_buffer()
    
    def reset_input_buffer(self):
        self.buffer.clear()
        
        for packet in FEEDBACK_PACKETS:
            length = packet['length']
            num_floats = length // 4 

            for x in [START_BYTE, packet['header']] + list(struct.pack(f'<{num_floats}f', *(i for i in range(num_floats)))):
                self.buffer.append(x)

        self.in_waiting = len(self.buffer)

    def flush(self):
        self.in_waiting = 0
        self.reset_input_buffer()  # Refill the buffer for testing

    def read(self, size):
        data = bytearray()
        
        # Read 'size' bytes safely by popping from the front and rotating to the back
        for _ in range(min(size, len(self.buffer))):
            byte = self.buffer.popleft()
            data.append(byte)
            self.buffer.append(byte) # Simulates continuous circular hardware stream
            
        self.in_waiting = len(self.buffer)
        return bytes(data)

    def write(self, data):
        self.logger.info(f"Mock serial write: {data.hex()}")

    def close(self):
        self.logger.info("Mock serial port closed")
