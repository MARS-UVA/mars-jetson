from serial_node.feedback_mappings import START_BYTE, FEEDBACK_PACKETS

class SerialHandler:
    def __init__(self, port, baudrate, timeout=1, mock_serial=False):
        if mock_serial:
            from serial_node.mock_serial import Serial
        else:
            from serial import Serial

        self.ser = Serial(port, baudrate, timeout=timeout)
        self.header_length_map = {packet['header']: packet['length'] for packet in FEEDBACK_PACKETS}
        self.ser.flush()

    def read(self, logger=None):
        if self.ser.in_waiting == 0:
            if logger:
                logger.debug("No data waiting in the serial buffer.")
            return None
        
        start_byte = self.ser.read(1)
        if start_byte != bytes([START_BYTE]):
            if logger:
                logger.warning(f"Invalid packet start: {start_byte.hex()}. Expected {hex(START_BYTE)}.")
            return None
        
        header_byte = self.ser.read(1)
        if not header_byte:
            if logger:
                logger.warning("Failed to read header byte from serial.")
            return None
        
        header = header_byte[0]
        length = self.header_length_map.get(header, None)
        if length is None:
            if logger:
                logger.warning(f"Unknown header received: {hex(header)}. No length mapping found.")
            return None

        data = self.ser.read(length)
        if len(data) != length:
            if logger:
                logger.warning(f"Expected {length} bytes of data, but received {len(data)} bytes.")
            return None
        
        if logger:
            logger.debug(f"Received packet: Header: {hex(header)}, Length: {length}, Data: {data.hex()}")

        return header, data

    def write(self, header, payload):
        packet = bytes([START_BYTE, header, len(payload)]) + payload
        self.ser.write(packet)
        if self.ser:
            self.ser.flush()

    def __del__(self):
        if self.ser:
            self.ser.close()
