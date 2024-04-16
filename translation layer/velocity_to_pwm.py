#import rclpy
#from rclpy.node import Node
#from geometry_msgs.msg import Twist
#from std_msgs.msg import UInt8
#from machine import Pin, PWM

#motor = PWM(Pin(6), freq=100, duty_u16=32768)
#servo = PWM(Pin(7), freq=100, duty_u16=32768)


def linvel_to_pwm(velocity, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle):
    
    # Convert a velocity command to a PWM signal.

    # Parameters:
    # velocity (float): The linear x velocity command from cmd_vel, which is of type TwistStamped.
    # max_velocity (float): The maximum possible linear x velocity (set in ros2_ack.xacro).
    # min_pwm_duty_cycle (float): The minimum possible PWM duty cycle in ns.
    # max_pwm_duty_cycle (float): The maximum possible PWM duty cycle in ns.

    # Returns:
    # float: The corresponding PWM signal.
    
    # Ensure the velocity is within the expected range
    velocity = max(min(velocity, max_velocity), -max_velocity)

    # Map the velocity to a PWM signal
    pwm_duty_cycle = ((velocity / max_velocity) * (max_pwm_duty_cycle - min_pwm_duty_cycle) / 2) + ((max_pwm_duty_cycle + min_pwm_duty_cycle) / 2)

    return pwm_duty_cycle

def angvel_to_pwm(velocity, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle):
    
    #Convert a velocity command to a PWM signal.

    # Parameters:
    # velocity (float): The linear x velocity command from cmd_vel, which is of type TwistStamped.
    # max_velocity (float): The maximum possible linear x velocity (set in ros2_ack.xacro).
    # min_pwm_duty_cycle (float): The minimum possible PWM duty cycle in ns.
    # max_pwm_duty_cycle (float): The maximum possible PWM duty cycle in ns.

    # Returns:
    # float: The corresponding PWM signal.
    
    # Ensure the velocity is within the expected range
    velocity = max(min(velocity, max_velocity), -max_velocity)

    # Map the velocity to a PWM signal
    pwm_duty_cycle = ((velocity / max_velocity) * (max_pwm_duty_cycle - min_pwm_duty_cycle) / 2) + ((max_pwm_duty_cycle + min_pwm_duty_cycle) / 2)

    return pwm_duty_cycle


velocity = 20.0                    # example linear velocity
max_velocity = 15.0                 # define maximum linear velocity
min_pwm_duty_cycle = 1000000        # define minimum duty cycle
max_pwm_duty_cycle = 2000000        # define maximum duty cycle
motor_duty_cycle = 0                  # initalize as an int


# conversion complete, send to DC Motor
motor_duty_cycle = linvel_to_pwm(velocity, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle)
print(motor_duty_cycle)
