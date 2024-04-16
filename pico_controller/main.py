import sys
import utime

sys.path.append('./hardware')

from servo import Servo  # Import the Servo class from servo.py
from motor import Motor  # Import the Motor class from motor.py
from lidar import Lidar  # Import the Lidar class from lidar.py
from ir_sensor import IRSensor  # Import the IRSensor class from ir_sensor.py

lidar = Lidar()
servo = Servo()
motor = Motor()
ir_sensor = IRSensor()

def test_motor(speed):
    print("Forward")
    motor.forward(speed)
    utime.sleep(3)

    print("Reverse")
    motor.reverse(speed)
    utime.sleep(3)

    print("Stop")
    motor.stop()
    utime.sleep(3)


def test_servo(angle):
    print("Straight")
    servo.straight()  # Adjust angle as needed
    utime.sleep(3)

    print("Turn Right")
    servo.right(angle)  # Assuming straight is a default position
    utime.sleep(5)

    print("Turn Left")
    servo.left(angle + 1)  # Adjust angle as needed
    utime.sleep(3)


def test_lidar():
    print("Starting Lidar Motor")
    lidar.start_motor()
    print("Reading Data")
    for i in range(10):
        measurements = lidar.get_measurement_data()
        print("Measurement Data: ", measurements)


while True:
    print("Running Motor Test")
    test_motor(0.5)
    print("Running Servo Test")
    test_servo(0.5)
    print("Running Lidar Test")
    test_lidar()
