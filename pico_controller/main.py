import sys
import machine
import utime
sys.path.append('./hardware')

from servo import Servo  # Import the Servo class from servo.py
from motor import Motor  # Import the Servo class from motor.py
from lidar import Lidar

while True:
    print("Turn Right")
    Servo.straight()
    utime.sleep(5)
    Motor.forward(0.3)
    Servo.right(0.75)
    utime.sleep(3)  # Pause for 1 second
    print("Turn Left")
    Motor.forward(0.5)
    Servo.left(1.5)
    utime.sleep(3)
    print("Move Backward")
    Servo.straight()
    Motor.reverse(0.5)
    utime.sleep(3)
    print("Move Forward")
    Servo.straight()
    Motor.forward(0.5)