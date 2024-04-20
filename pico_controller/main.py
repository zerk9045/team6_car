import sys
import utime

sys.path.append('./hardware')

from servo import Servo
from motor import Motor
from lidar import Lidar
#from ir_sensor import IRSensor

lidar = Lidar()
servo = Servo()
motor = Motor()
#ir_sensor = IRSensor()


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
    servo.straight()
    utime.sleep(3)

    print("Turn Right")
    servo.right(angle)
    utime.sleep(5)

    print("Turn Left")
    servo.left(angle + 1)
    utime.sleep(3)
    
    print("Straight")
    servo.straight()
    utime.sleep(3)


def test_lidar():
    print("Starting Lidar Motor")
    lidar.start_motor()
    print("Reading Data")
    for i in range(10):
        measurements = lidar.get_measurement_data()
        print("Measurement Data: ", measurements)


#def test_ir_sensor():
    #while True:
        #print("IR Sensor Interrupts:")
        #print(ir_sensor.sensor_interrupts)
        #utime.sleep(2)  # Print every 2 seconds


# Uncomment the test functions you want to run
if __name__ == "__main__":
    test_motor(0.4)
    test_servo(0.4)
    # test_lidar()
    # test_ir_sensor()