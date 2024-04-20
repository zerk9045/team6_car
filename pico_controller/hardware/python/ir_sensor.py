import machine
import sys
from machine import Timer
sys.path.append('../config')
from config.pin_config import Pin


class IRSensor:
    def __init__(self):
        self.sensor = Pin.IR_SENSOR_PIN
        self.sensor.irq(trigger=machine.Pin.IRQ_FALLING, handler=self.do_interrupt)
        self.sensor_interrupts = 0

        self.timer = machine.Timer(-1)
        self.timer.init(period=1000, mode=machine.Timer.PERIODIC, callback=self.timer_action)

    def do_interrupt(self, pin):
        self.sensor_interrupts += 1
        
    def timer_action(self, timer):
        speed = self.sensor_interrupts
        print("Counts per second =", speed)
        self.sensor_interrupts = 0

ir_sensor = IRSensor()
