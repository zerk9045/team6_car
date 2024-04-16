import serial
import select


class SerialCommunication:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(self.port, self.baudrate)

        # Set up the poll object
        self.poll_obj = select.poll()
        self.poll_obj.register(self.serial, select.POLLIN)

    def read_data(self):
        # Wait for input on serial port
        poll_results = self.poll_obj.poll(1)  # 1ms timeout for polling
        if poll_results:
            # Read data from serial port
            data = self.serial.readline().decode().strip()
            return data
        else:
            # No data received
            return None

    def write_data(self, data):
        # Write data to serial port
        self.serial.write(data.encode())

    def close(self):
        # Close serial port
        self.serial.close()
