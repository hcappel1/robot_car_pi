import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time
import board
from adafruit_motorkit import MotorKit
from os import geteuid

kit = MotorKit(i2c=board.I2C())
kit.motor1.throttle = 0.6
kit.motor2.throttle = 0.6
kit.motor3.throttle = 0.6
kit.motor4.throttle = 0.6

class EncoderPublisher(Node):

	def __init__(self):
		super().__init__('encoder_count_node')
		self.publisher = self.create_publisher(Int32, 'encoder_count_m1', 100)
		self.publisher = self.create_publisher(Int32, 'encoder_count_m2', 100)
		self.publisher = self.create_publisher(Int32, 'encoder_count_m3', 100)
		self.publisher = self.create_publisher(Int32, 'encoder_count_m4', 100)

		self.stateLast_m1 = GPIO.input(17)
		self.stateLast_m2 = GPIO.input(18)
		self.stateLast_m3 = GPIO.input(27)
		self.stateLast_m4 = GPIO.input(22)

		self.stateCount_m1 = 0
		self.stateCount_m2 = 0
		self.stateCount_m3 = 0
		self.stateCount_m4 = 0

		self.rotationCount_m1 = 0
		self.rotationCount_m2 = 0
		self.rotationCount_m3 = 0
		self.rotationCount_m4 = 0
		
		self.statesPerRotation = 40		
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def timer_callback(self):
		self.stateCurrent = GPIO.input(17)
		if self.stateCurrent != self.stateLast:
			self.stateLast = self.stateCurrent
			self.stateCount += 1
			msg = Int32()
			msg.data = self.stateCount
			self.publisher.publish(msg)
			#self.get_logger().info('%d' % self.stateCount)
		
		if self.stateCount == self.statesPerRotation:
			self.rotationCount += 1
			self.stateCount = 0

def main(args=None):
	rclpy.init(args=args)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(17,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	encoder_publisher = EncoderPublisher()
	rclpy.spin(encoder_publisher)
	encoder_publisher.destroy_node()
	rclpy.shutdown()
	
		
		


if __name__ == '__main__':
	main()