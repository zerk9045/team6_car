#import rclpy
#from rclpy.node import Node
#from geometry_msgs.msg import TwistStamped
#from std_msgs.msg import UInt8
#from machine import Pin, PWM

#motor = PWM(Pin(6), freq=100, duty_u16=32768)
#servo = PWM(Pin(7), freq=100, duty_u16=32768)


def linvel_to_pwm(linvel, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle):
    
    # Convert a velocity command to a PWM signal.

    # Parameters:
    # linvel (float): The linear x velocity command from cmd_vel, which is of type TwistStamped.
    # max_velocity (float): The maximum possible linear x velocity (set in ros2_ack.xacro).
    # min_pwm_duty_cycle (float): The minimum possible PWM duty cycle in ns.
    # max_pwm_duty_cycle (float): The maximum possible PWM duty cycle in ns.

    # Returns:
    # motor_duty_cycle (int): The corresponding PWM signal.
    
    # Ensure linvel is within the expected range
    linvel = max(min(linvel, max_velocity), -max_velocity)

    # Map linvel to a PWM signal
    motor_duty_cycle = ((linvel / max_velocity) * (max_pwm_duty_cycle - min_pwm_duty_cycle) // 2) + ((max_pwm_duty_cycle + min_pwm_duty_cycle) // 2)

    return motor_duty_cycle

def angvel_to_pwm(angvel, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle):
    
    # Convert a velocity command to a PWM signal.

    # Parameters:
    # angvel (float): The angular z velocity command from cmd_vel, which is of type TwistStamped.
    # max_velocity (float): The maximum possible angular velocity (set in ros2_ack.xacro).
    # min_pwm_duty_cycle (int): The minimum possible PWM duty cycle in ns.
    # max_pwm_duty_cycle (int): The maximum possible PWM duty cycle in ns.

    # Returns:
    # servo_duty_cycle (int): The corresponding PWM signal.
    
    # Ensure angvel is within the expected range
    angvel = max(min(angvel, max_velocity), -max_velocity)

    # Map angvel to a PWM signal
    servo_duty_cycle = ((angvel / max_velocity) * (max_pwm_duty_cycle - min_pwm_duty_cycle) // 2) + ((max_pwm_duty_cycle + min_pwm_duty_cycle) // 2)

    return servo_duty_cycle

angvel = 15.0                       # example angular velocity
linvel = 10.0                       # example linear velocity
max_velocity = 15.0                 # define maximum linear velocity
min_pwm_duty_cycle = 1000000        # define minimum duty cycle
max_pwm_duty_cycle = 2000000        # define maximum duty cycle
motor_duty_cycle = 0                # initialize as an int
servo_duty_cycle = 0                # initialize as an int

# conversion complete, send to DC Motor
motor_duty_cycle = linvel_to_pwm(linvel, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle)
print("motor duty cycle =" , motor_duty_cycle)

servo_duty_cycle = angvel_to_pwm(angvel, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle)
print("servo duty cycle =" , servo_duty_cycle)
