import machine
import sys

sys.path.append('../config')
from config.pin_config import Pin


class IRSensor:
    def __init__(self):
        self.sensor = Pin.IR_SENSOR_PIN
        self.sensor.irq(machine.handler(self.do_interrupt), machine.trigger(machine.Pin.IRQ_FALLING))
        self.sensor_interrupts = 0

        self.timer = machine.Timer()
        self.timer.init(machine.mode(machine.Timer.PERIODIC), machine.period(1000), machine.callback(self.timer_action))

    def do_interrupt(self):
        self.sensor_interrupts += 1

    def timer_action(self, timer):
        speed = self.sensor_interrupts
        print("Counts per second =", speed)
        self.sensor_interrupts = 0
