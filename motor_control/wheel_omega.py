import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

import math


class WheelAngVel(Node):

	def __init__(self):
		super().__init__('wheel_ang_vel_node')
		self.prev_encoder_count = 0
		self.encoder_count = 0

		self.wheel1_omega = 0

		self.subscription = self.create_subscription(
			Int32,
			'encoder_count',
			self.listener_cb,
			10
			)
		self.subscription

		self.timer = self.create_timer(0.2, self.timer_callback)

	def timer_callback(self):
		if (self.encoder_count >= self.prev_encoder_count):
			encoder_diff = self.encoder_count - self.prev_encoder_count
		else:
			encoder_diff = self.encoder_count + (40 - self.prev_encoder_count)

		self.wheel1_omega = (encoder_diff*math.pi/20)/0.2 
		self.prev_encoder_count = self.encoder_count
		self.get_logger().info('wheel 1 ang vel: %f' % self.wheel1_omega)

	def listener_cb(self, msg):
		self.encoder_count = msg.data
		#self.get_logger().info('receiving encoder data: %d' % self.encoder_count)

def main(args=None):
	rclpy.init(args=args)

	wheel_ang_vel = WheelAngVel()

	rclpy.spin(wheel_ang_vel)

	wheel_ang_vel.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


