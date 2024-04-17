import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8
#from machine import Pin, PWM


# set inital parameters for PWM signals for motor and servo
# both must be 100 Hz
#motor = PWM(Pin(6), freq=100, duty_u16=32768)
#servo = PWM(Pin(7), freq=100, duty_u16=32768)

angular_vel = 15.0                  # example angular velocity
linear_vel = 10.0                   # example linear velocity
max_velocity = 15.0                 # define maximum linear velocity
min_pwm_duty_cycle = 1000000        # define minimum duty cycle
max_pwm_duty_cycle = 2000000        # define maximum duty cycle

class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)





#def callback(msg)
    
    # take in velocity messages from cmd_vel
 #   linear_vel = msg.linear.x
  #  angular_vel = msg.angular.z
    
    # Ensure linear_vel is within the expected range
   # linear_vel = max(min(linear_vel, max_velocity), -max_velocity)

    # Ensure angular_vel is within the expected range
    #angular_vel = max(min(angular_vel, max_velocity), -max_velocity)

    # Map linear_vel to a PWM signal
    #motor_duty_cycle = ((linear_vel / max_velocity) * (max_pwm_duty_cycle - min_pwm_duty_cycle) // 2) + ((max_pwm_duty_cycle + min_pwm_duty_cycle) // 2)
    
    # Map angular_vel to a PWM signal
    #servo_duty_cycle = ((angular_vel / max_velocity) * (max_pwm_duty_cycle - min_pwm_duty_cycle) // 2) + ((max_pwm_duty_cycle + min_pwm_duty_cycle) // 2)

    # set new duty cycles for motor and servo
#    motor.duty_ns(motor_duty_cycle)
#    servo.duty_ns(servo_duty_cycle)



def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)
    
    
# conversion complete, send to DC Motor
#motor_duty_cycle = linear_vel_to_pwm(linear_vel, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle)
#print("motor duty cycle =" , motor_duty_cycle)

#servo_duty_cycle = angular_vel_to_pwm(angular_vel, max_velocity, min_pwm_duty_cycle, max_pwm_duty_cycle)
#print("servo duty cycle =" , servo_duty_cycle)
