import machine

class PinConfig:
    def __init__(self):
        # Other pin configurations
        self.PWM_FREQUENCY = 100  # PWM frequency in Hz

        # Pin Mappings for VNH5019A-E controls
        self.INA_PIN = machine.Pin(10, machine.Pin.OUT)  # Motor driver control pin
        self.INB_PIN = machine.Pin(11, machine.Pin.OUT)  # Motor driver control pin

        # Pin mappings for motors and servos
        self.WHEEL_MOTOR_PIN= machine.PWM(machine.Pin(14))  # GPIO pin 14 for motor control (PWM)
        self.SERVO_PIN = machine.PWM(machine.Pin(15))  # GPIO pin 15 for servo control (PWM)

        # Pin mappings for LiDAR
        self.UART_TX_PIN = machine.Pin(12)
        self.UART_RX_PIN = machine.Pin(13)
        self.LIDAR_MOTOR_PIN = machine.PWM(machine.Pin(9))

# Create an instance of PinConfig for easy access
Pin = PinConfig()


