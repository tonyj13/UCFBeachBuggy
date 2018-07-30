#!/usr/bin/env python

# Tony Jimogaon
# Senior Design 2 2018
# Group 1

# ==========================
#	Motor Control Node
# ==========================

from geometry_msgs.msg import Twist
import roslib
import rospy
import math
import RPi.GPIO as GPIO

# Global PWM variable

# Print Colors

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def callback(data):
	# Log to screen if heard
	#print "Motor Control: Received linear.x=", data.linear.x, ", angular.z = ", data.angular.z

	# Time stuff
	rate = rospy.Duration(abs(data.linear.x))

	# Move Forward
	if data.linear.x > 0:
		#print bdata.linear.xcolors.OKBLUE + "Buggy Control: Moving Forward..." + bcolors.ENDC
		forward(data.linear.x)
	
	# Move backward
	if data.linear.x < 0:
		#print bcolors.OKBLUE + "Buggy Control: Moving Backward..." + bcolors.ENDC
		backward(data.linear.x)
		
	# Move right
	if data.angular.z < 0:
		#print bcolors.OKBLUE + "Buggy Control: Moving Right..." + bcolors.ENDC
		right(data.angular.z)
		
	# Move left
	if data.angular.z > 0:
		#print bcolors.OKBLUE + "Buggy Control: Moving Left..." + bcolors.ENDC
		left(data.angular.z)
		
	# Stop
	if data.linear.x == 0 and data.angular.z == 0:
		#print bcolors.RED + "Buggy Control: Stopping..." + bcolors.ENDC
		stop()

def listener():
	# Initialize Node
	rospy.init_node('listener_control', anonymous=True)

	# Subscribe to Node "controller_buggy"
	rospy.Subscriber("controller_buggy", Twist, callback)

	# Prevents closing of python script until node is terminated
	rospy.spin()

def initGPIO():
	# Use physical pin numbers
	GPIO.setmode(GPIO.BOARD)
	
	# Use GPIO physical Pin # 3 and 5
	GPIO.setup(3, GPIO.OUT)	
	GPIO.setup(5, GPIO.OUT)

	GPIO.setup(11, GPIO.OUT)	
	GPIO.setup(13, GPIO.OUT)
	
	# 2k Hz PWM on Pin # 3 and 5
	global pwmL
	pwmL = GPIO.PWM(3, 2000)
	global pwmR
	pwmR = GPIO.PWM(5, 2000)
	
	# Init with 0% duty cycle (1.6V)
	pwmL.start(0)
	pwmR.start(0)
	
def forward(percent):
	power = int(round(percent/100 * 100))
	GPIO.output(11, GPIO.HIGH)
	GPIO.output(13, GPIO.HIGH)
	pwmL.ChangeDutyCycle(power)
	pwmR.ChangeDutyCycle(power)
	
def backward():
	power = int(round(percent/100 * 100))
	pwmL.ChangeDutyCycle(power)
	pwmR.ChangeDutyCycle(power)
	
def right(percent):
	absPercent = math.fabs(percent)
	power = int(round(absPercent/100 * 100))
	GPIO.output(13, GPIO.LOW)
	GPIO.output(11, GPIO.HIGH)
	pwmL.ChangeDutyCycle(power)
	pwmR.ChangeDutyCycle(power)

def left(percent):
	power = int(round(percent/100 * 100))
	GPIO.output(11, GPIO.LOW)
	GPIO.output(13, GPIO.HIGH)
	pwmL.ChangeDutyCycle(power)
	pwmR.ChangeDutyCycle(power)
	
def stop():
	pwmL.ChangeDutyCycle(0)
	pwmR.ChangeDutyCycle(0)
	
def shutdown():
	print bcolors.WARNING + "Motor Control: Shutting Down" + bcolors.ENDC
	stop()
	# Release GPIO pins
	#GPIO.cleanup()
	
def test():
	while True:
		print "Moving Forward"
		forward()
		rospy.sleep(5.)
		print "Moving Backward"
		backward()
		rospy.sleep(5.)
		print "Stopping"
		stop()
		rospy.sleep(5.)

if __name__ == '__main__':
	initGPIO()
	rospy.on_shutdown(shutdown)
	listener()
	#test()

