import sys
import machine
sys.path.append('../config')
from config.pin_config import Pin

class Motor:
    @staticmethod
    def forward(speed):
        Pin.INA_PIN.value(1)  # Set INA to high
        Pin.INB_PIN.value(0)  # Set INB to low
        Pin.WHEEL_MOTOR_PIN.duty_ns(int(speed * 3000000))  # Set PWM duty cycle #min 0.9ms

    @staticmethod
    def reverse(speed):
        Pin.INA_PIN.value(0)  # Set INA to low
        Pin.INB_PIN.value(1)  # Set INB to high
        Pin.WHEEL_MOTOR_PIN.duty_ns(int(speed * 3000000))  # Set PWM duty cycle

    @staticmethod
    def stop():
        Pin.INA_PIN.value(0)  # Stop both INA and INB
        Pin.INB_PIN.value(0)
        Pin.WHEEL_MOTOR_PIN.duty_u16(0)  # Stop PWM
