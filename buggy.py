#!/usr/bin/env python

# Tony Jimogaon
# Senior Design 2 2018
# Group 1

# ==========================
#	Motor Control Node
# ==========================

import rospy
import math
import numpy
import curses

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from threading import Thread, Lock, ThreadError

from time import sleep, time

USE_SCREEN = True

rangeData = []
tempList = []
acquired = False
gpsConflictFront = False
gpsConflictLeft = False
gpsConflictRight = False

class bcolors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  RED = '\033[91m'
  ENDC = '\033[0m'
  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'
  
def gpsCallback(receivedCallback):
	if receivedCallback == 2:
		gpsConflictFront = True
	if receivedCallback == 1:
		gpsConflictLeft = True
	if receivedCallback == 3:
		gpsConflictRight = True
	else:
		gpsConflictFront = False
		gpsConflictLeft = False
		gpsConflictRight = False

def lidarCallback(receivedCallback):
	global acquired
	global msg
	msg = receivedCallback
	
	# Release the locked thread
	try:
		lock.release()
		acquired = False
	except ThreadError:
		pass
		
def servoCallback(receivedCallback):
	global angle
	angle = receivedCallback

def buggyControl():
	global rightToLeft
	global acquired
	global rangeData
	global tempList
	global angle
	del rangeData [:]
	del tempList [:]
	servoMsg = Int16() 
	
	# Move Servo right to left
	for x in range(0, 181, 3):
		acquired = lock.acquire(True)
		rightToLeft = True
		
		# Publish to servo
		servoMsg.data = x
		pubServo.publish(servoMsg)
		#sleep(0.02)
		# Wait for servo
		while x != angle.data:
			pass
		
		# Grab data from lidar
		lidarProcessor(msg)
		
	# Obstacle avoidance
	sense()
	# Clear list
	del rangeData [:]
		
	# Move Servo left to right
	for x in range(180, -1, -3):
		acquired = lock.acquire(True)
		rightToLeft = False
		
		# Publish to servo
		servoMsg.data = x
		pubServo.publish(servoMsg)
		#sleep(0.02)
		# Wait for servo
		while x != angle.data:
			pass
		
		# Grab data from lidar
		lidarProcessor(msg)
		
	rangeData = list(reversed(tempList))
	# Obstacle avoidance
	sense()
	# Clear list
	del tempList [:]
	del rangeData [:]
	
def lidarProcessor(msg):
	if(rightToLeft):
			rangeData.append(msg.ranges[0])
	else:
			tempList.append(msg.ranges[0])
	
	
	if USE_SCREEN:
		screen.addstr(3, 0, "LiDAR: Current Value: " + "%.5f" %msg.ranges[0] + "m")
		screen.refresh()
	else:
		print "LiDAR: Current Value:", msg.ranges[0], "\n"
		
def sense():
	l = len(rangeData)
	# init Twist
	twistMsg = Twist()

	# Divide list in to three sections
	#	
	first = rangeData[:int(l * 5/12)]
	#second = rangeData[(l/3 + 1) : (2 * l/3)]
	second = rangeData[int((l * 5/12) + 1) : int(l * 7/12)]
	#third = rangeData[((2 * l/3) + 1) :]
	third = rangeData[int((l * 7/12) + 1) :]

	# Avoid obstacles start
	# Filter > 12m for sections
	cleanFirst = [x for x in first if x <= 12]
	cleanSecond = [x for x in second if x <= 12]
	cleanThird = [x for x in third if x <= 12]

	# Min of sections
	try:
		cleanrightMin = min(cleanFirst)
	except ValueError:
		cleanrightMin = float('NaN')
	try:
		cleanmiddleMin = min(cleanSecond)
	except ValueError:
		cleanmiddleMin = float('NaN')
	try:
		cleanleftMin = min(cleanThird)
	except ValueError:
		cleanleftMin = float('NaN')
		
	# Print depending on how fancy you want the output to be
	if USE_SCREEN:
		screen.addstr(0, 0, "LiDAR: Front cone object distance: " + "%.5f" % cleanmiddleMin + "m" + " " * 6)
		screen.addstr(1, 0, "LiDAR: Left cone object distance: " + "%.5f" %cleanleftMin + "m" + " " * 6)
		screen.addstr(2, 0, "LiDAR: Right cone object distance: " + "%.5f" %cleanrightMin + "m" + " " * 6)
		screen.refresh()
	else:
		print "LiDAR: Front cone object distance: ", cleanmiddleMin,"m\n"
		print "LiDAR: Left cone object distance: ", cleanleftMin,"m\n"
		print "LiDAR: Right cone object distance: ", cleanrightMin,"m\n"
	
	# If there is no obstacle within 2.0 m
	#   of the front cone, drive forward at 0.5. 
	if ((cleanmiddleMin > 2) or (math.isnan(cleanmiddleMin))) and not(gpsConflictFront):
		twistMsg.linear.x = 20
		twistMsg.angular.z = 0
		pubTwist.publish(twistMsg)
		printBuggyStatus(0)
	#If there is an obstacle there, 
	#  check right for obstacles and turn right if none.
	elif ((cleanrightMin > 1.25) or (math.isnan(cleanrightMin))) and not(gpsConflictRight):
		twistMsg.linear.x = 0
		twistMsg.angular.z = -10
		pubTwist.publish(twistMsg)
		printBuggyStatus(1)
	#If there is a right obstacle, 
	#  check left for obstacles and turn left if none.
	elif ((cleanleftMin > 1.25) or (math.isnan(cleanleftMin))) and not(gpsConflictLeft):
		twistMsg.linear.x = 0
		twistMsg.angular.z = 10
		pubTwist.publish(twistMsg)
		printBuggyStatus(2)
	#Else stop.
	else:
		twistMsg.linear.x = 0
		twistMsg.angular.z = 0
		pubTwist.publish(twistMsg)
		printBuggyStatus(3)
		
def printBuggyStatus(i):
	if USE_SCREEN:
		if i == 0:
			screen.addstr(4, 0, "Buggy Control: Moving Forward..." + " " * 6)
		elif i == 1:
			screen.addstr(4, 0, "Buggy Control: Moving Right..." + " " * 6)
		elif i == 2:
			screen.addstr(4, 0, "Buggy Control: Moving Left..." + " " * 6)
		elif i == 3:
			screen.addstr(4, 0, "Buggy Control: Stopping..." + " " * 6)
		
		screen.refresh()
	else:
		if i == 0:
			print bcolors.RED + "Buggy Control: Moving Forward..." + bcolors.ENDC
		elif i == 1:
			print bcolors.RED + "Buggy Control: Moving Right..." + bcolors.ENDC
		elif i == 2:
			print bcolors.RED + "Buggy Control: Moving Left..." + bcolors.ENDC
		elif i == 3:
			print bcolors.RED + "BBuggy Control: Stopping..." + bcolors.ENDC

def shutdown():
	curses.endwin()
	servoMsg = Int16()
	servoMsg.data = 0
	pubServo.publish(servoMsg)
	
	twistMsg = Twist()
	twistMsg.linear.x = 0
	twistMsg.angular.z = 0
	pubTwist.publish(twistMsg)
	printBuggyStatus(3)
	print bcolors.WARNING + "Buggy Control: Shutting Down" + bcolors.ENDC
	
# Initialize everything
def init():
	global pubLidar
	global pubServo
	global pubTwist
	global screen
	global thread
	global subLidarThread
	global subServoThread
	global subGpsThread
	global lock
	
	rospy.init_node('buggy_control', anonymous=True)
	pubLidar = rospy.Publisher('toggle_lidar', Bool, queue_size = 1)
	pubServo = rospy.Publisher('toggle_servo', Int16, queue_size = 1)
	pubTwist = rospy.Publisher('controller_buggy', Twist, queue_size = 1)	
	screen = curses.initscr()

	subLidarThread = Thread(target = sub)
	subLidarThread.daemon = True
	subServoThread = Thread(target = sub2)
	subServoThread.daemon = True
	subGpsThread = Thread(target = sub3)
	subGpsThread.daemon = True
	
def sub():
	subLidar = rospy.Subscriber('range_data', LaserScan, callback = lidarCallback, queue_size = 1)
	
def sub2():
	subServo = rospy.Subscriber('servo_status', Int16, callback = servoCallback, queue_size = 1)
	
def sub3():
	subGps = rospy.Subscriber('gps_status', Int16, callback = gpsCallback, queue_size = 1)
	
if __name__ == '__main__':
	init()
	subLidarThread.start()
	subServoThread.start()
	subGpsThread.start()
	rospy.on_shutdown(shutdown)
	#import gps
	while not rospy.is_shutdown():
		lock = Lock()
		thread = Thread(target = buggyControl)
		thread.daemon = True
		thread.start()
		thread.join()
	
	# Prevents closing of python script until node is terminated
	rospy.spin()
