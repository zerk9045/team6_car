import sys
import machine
sys.path.append('../config')
from config.pin_config import Pin

# Left > 1.5ms Right < 1.5ms
class Servo:
    @staticmethod
    def left(angle):
        Pin.SERVO_PIN.duty_ns(int(angle * 1500000))

    @staticmethod
    def right(angle):
        Pin.SERVO_PIN.duty_ns(int(angle * 1500000))

    @staticmethod
    def straight():
        Pin.SERVO_PIN.duty_ns(1500000)
