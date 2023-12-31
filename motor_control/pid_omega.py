import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import math
import board
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())
kit.motor1.throttle = 0.0
kit.motor2.throttle = 0.0
kit.motor3.throttle = 0.0
kit.motor4.throttle = 0.0

class OmegaPID(Node):

	def __init__(self):
		super().__init__('encoder_count_node')

		self.m1_throttle = 0.8
		self.kp = 0.15
		self.ki = 0.1
		self.kd = 0.03
		self.m1_error = 0
		self.m1_cum_error = 0
		self.m1_cum_error_prev = 0
		self.m1_rate_error = 0
		self.m1_last_error = 0
		self.m1_output_voltage = 0
		self.m1_max_voltage = 11.1
		self.m1_input = 0
		self.dt = 0.01

		self.omega_m1_meas = 0
		self.omega_m1_ref = 10

		self.omega_subscription = self.create_subscription(
			Float32,
			'wheel_velocities',
			self.omega_motors_cb,
			10
			)
		self.omega_subscription

		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.timer2 = self.create_timer(5, self.timer2_callback)

	def timer_callback(self):
		self.m1_error = self.omega_m1_ref - self.omega_m1_meas
		self.m1_cum_error = self.m1_cum_error_prev + self.m1_error*self.dt
		self.m1_rate_error = (self.m1_error - self.m1_last_error)/self.dt
		self.m1_output_voltage = self.kp*self.m1_error + self.ki*self.m1_cum_error + self.kd*self.m1_rate_error

		if (self.m1_output_voltage > self.m1_max_voltage):
			self.m1_output_voltage = self.m1_max_voltage
			self.m1_cum_error = self.m1_cum_error_prev

		self.m1_last_error = self.m1_error
		self.m1_cum_error_prev = self.m1_cum_error

		self.m1_input = self.m1_output_voltage/self.m1_max_voltage
		if (self.m1_input > 1):
			self.m1_input = 1

		if (self.m1_input < -1):
			self.m1_input = -1


		kit.motor1.throttle = self.m1_input
		#self.m1_throttle = self.m1_throttle #+ 0.001
		
		self.get_logger().info('wheel 1 throttle: %f' % self.m1_input)
		self.get_logger().info('wheel 1 vel: %f' % self.omega_m1_meas)

	def timer2_callback(self):
		if (self.omega_m1_ref < 50):
			self.omega_m1_ref = self.omega_m1_ref + 10
		else:
			self.omega_m1_ref = 0

	def omega_motors_cb(self, msg):
		self.omega_m1_meas = msg.data

def main(args=None):
	rclpy.init(args=args)
	
	omega_pid = OmegaPID()
	try:
		rclpy.spin(omega_pid)
	except KeyboardInterrupt:
		kit.motor1.throttle = 0.0
		omega_pid.destroy_node()
		rclpy.shutdown()
	
		

if __name__ == '__main__':
	main()