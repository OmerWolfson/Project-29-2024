#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

import RPi.GPIO as GPIO
from time import sleep

from project29_interfaces.msg import SerComStruct


class PwmSubNode(Node):
	def __init__(self):
		super().__init__('ser_com_subscriber')
		self.get_logger().info("Node initiated")
		self.sub_ = self.create_subscription(SerComStruct, 'motor_throttle_control', self.listener_callback, 10)
		
		# Pin layout
		self.pwm_l = 18         # pwm of left/right wheel
		self.pwm_r = 13
		self.is_reverse_l = 15  # reverse state of left/right wheel
		self.is_reverse_r = 19
		self.gear_l = 14        # gear state of left/right wheel
		self.gear_r = 26

		# GPIO setup
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pwm_l,GPIO.OUT)
		GPIO.setup(self.pwm_r,GPIO.OUT)
		GPIO.setup(self.is_reverse_l,GPIO.OUT)
		GPIO.setup(self.is_reverse_r,GPIO.OUT)
		GPIO.setup(self.gear_l,GPIO.OUT)
		GPIO.setup(self.gear_r,GPIO.OUT)

		# Initialize PWM
		self.left_throttle = GPIO.PWM(self.pwm_l, 500) # setting 500 Hz
		self.right_throttle = GPIO.PWM(self.pwm_r, 500)
		self.left_throttle.start(0)
		self.right_throttle.start(0)
		self.prev_msg = True 

	def listener_callback(self, msg):		
		# Set PWM values
		self.left_throttle.ChangeDutyCycle(msg.pwm_l)
		self.right_throttle.ChangeDutyCycle(msg.pwm_r)
		if msg.pwm_l > 50 and msg.pwm_r > 50:
			self.get_logger().info(f"PWM:\nLeft: {msg.pwm_l}, Right: {msg.pwm_r}")

		# Set reverse values
		GPIO.output(self.is_reverse_l, msg.is_reverse_dir_l)
		GPIO.output(self.is_reverse_r, msg.is_reverse_dir_r)
		if msg.is_reverse_dir_l and msg.is_reverse_dir_r:
			self.get_logger().info(f"Reverse direction:\nLeft : {msg.is_reverse_dir_l}, Right: {msg.is_reverse_dir_r}")

		# Set gear values
		GPIO.output(self.gear_l, msg.gear)
		GPIO.output(self.gear_r, msg.gear)
		if msg.gear is not self.prev_msg:
			self.get_logger().info(f"Gear changed to {msg.gear}")
			self.prev_msg = msg.gear
		

def main(args=None):
	rclpy.init(args=None)
	node = PwmSubNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()