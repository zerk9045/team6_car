import machine

sys.path.append('../config')
from config.pin_config import Pin

class Lidar:
    def __init__(self):
        self.lidar_uart = machine.UART(0, machine.baudrate(230400), machine.rx(Pin.UART_RX_PIN))
        self.motor_pwm = machine.PWM(machine.Pin(Pin.LIDAR_MOTOR_PIN))
        self.motor_pwm.freq(30000)  # Set PWM frequency to 30KHz (recommended)

    def control_motor_pwm(self, duty_cycle):
        """
        Control the motor PWM signal within specified conditions.

        Args:
        - duty_cycle (float): Duty cycle value (0-1) for the PWM signal.
        """
        if 0.45 < duty_cycle < 0.55:
            # Set PWM duty cycle only if within specified range
            self.motor_pwm.duty(int(duty_cycle * 1023))  # Convert duty cycle to PWM range (0-1023)
        else:
            # Duty cycle not in specified range, stop motor (set duty cycle to 0)
            self.motor_pwm.duty(0)

    def start_motor(self, duty_cycle=0.5):
        """
        Start spinning the LIDAR motor.

        Args:
        - duty_cycle (float): Duty cycle value (0-1) for motor PWM (default is 0.5).
        """
        self.control_motor_pwm(duty_cycle)  # Start motor with specified duty cycle

    def read_data(self,bytes):
        return self.lidar_uart.read(bytes)

    @staticmethod
    def parse_data_packet(data_packet):
        if len(data_packet) != 22:
            print("Invalid data packet length")
            return None

        # Parse data packet fields
        header = data_packet[0]
        ver_len = data_packet[1]
        speed = int.from_bytes(data_packet[2:4], 'big')  # 2 bytes for speed
        start_angle = int.from_bytes(data_packet[4:6], 'big') * 0.01  # 2 bytes for start angle
        data_points = []
        for i in range(6, 18, 3):
            data_point = int.from_bytes(data_packet[i:i + 3], 'big')  # 3 bytes for data point
            data_points.append(data_point)
        end_angle = int.from_bytes(data_packet[18:20], 'big') * 0.01  # 2 bytes for end angle
        timestamp = int.from_bytes(data_packet[20:22], 'big')  # 2 bytes for timestamp
        crc_check = data_packet[-1]

        return {
            'header': header,
            'ver_len': ver_len,
            'speed': speed,
            'start_angle': start_angle,
            'data_points': data_points,
            'end_angle': end_angle,
            'timestamp': timestamp,
            'crc_check': crc_check
        }

    @staticmethod
    def parse_measurement_data(data_points, start_angle, end_angle):
        len_data = len(data_points)
        if len_data == 0:
            return []

        step = (end_angle - start_angle) / (len_data - 1)
        angles = [start_angle + step * i for i in range(len_data)]

        measurements = []
        for i in range(len_data):
            distance_mm = int.from_bytes(data_points[i][:2], 'big')  # 2-byte distance value
            confidence = data_points[i][2]  # 1-byte confidence value
            angle = angles[i]
            measurements.append({'distance_mm': distance_mm, 'confidence': confidence, 'angle': angle})

        return measurements

    def get_measurement_data(self):
        # Read data from UART
        data = self.read_data(22)  # Assuming a data packet size of 22 bytes

        # Parse data packet
        data_packet = self.parse_data_packet(data)

        if data_packet:
            # Extract measurement data
            data_points = data_packet['data_points']
            start_angle = data_packet['start_angle']
            end_angle = data_packet['end_angle']

            # Process measurement data
            measurement_data = self.parse_measurement_data(data_points, start_angle, end_angle)
            return measurement_data
        else:
            print("Failed to parse data packet")
            return None