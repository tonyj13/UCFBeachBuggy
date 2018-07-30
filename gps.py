#!/usr/bin/env python

# Tony Jimogaon
# Senior Design 2 2018
# Group 1

# ==========================
#	GPS Node
# ==========================

import rospy
import math
import curses
import geopy
from geopy.distance import geodesic
from shapely.geometry import Polygon, Point
from math import degrees, atan2

from gps_common.msg import GPSFix
from std_msgs.msg import Int16

gpsConflict = Int16()

# Hardcoding....
# North of HEC Sidewalk Destination: 28.600767, -81.196848
phi2 = math.radians(28.600767)
lambda2 = math.radians(-81.196848)

# Off limit areas (grass)
leftGrass = Polygon(((28.600161, -81.197328),
										(28.600346, -81.197138),
										(28.600511, -81.197014),
										(28.600798, -81.196856),
										(28.600835, -81.197004),
										(28.600228, -81.197431)))
rightGrass = Polygon(((28.600129, -81.197286),
											(28.600308, -81.197119),
											(28.600481, -81.196986),
											(28.600832, -81.196793),
											(28.600804, -81.196670),
											(28.600087, -81.197212)))

currentHeading = float('NaN')

def gpsCallback(gpsMsg):
	phi1 = gpsMsg.latitude
	lambda1 = gpsMsg.longitude
	currentLoc = Point(phi1, lambda1)
	d = pythagorean(lambda1, phi1)
	
	print "Distance to target: ", d, "m (", d * 3.28084, "feet )"
	print "Current Location:", phi1, lambda1
	#screen.addstr(6, 0, "Distance to target: " + "%.3f" %d + " " * 6)
	#screen.addstr(7, 0, "Current Location:: " + "%.3f" %d + " " * 6)
	#screen.refresh()
	
	# Find if heading and nearby perimeters is going to collide with grass
	# Speed is in m/s. Check if speed is greater than 1mph
	print "Gps Heading Uncertainty: ", err_track
	print "Gps speed:", gpsMsg.speed
	if gpsMsg.speed > 0.2:
		# Check if any of the heading points are in the grass
		# Return status if it is
		print "hi there"
		
		point = findForwardPoint(gpsMsg, gpsMsg.track, 1)
		frontLat, frontLon = point.latitude, point.longitude
		frontPoint = Point(frontLat, frontLon)
		if frontPoint.within(leftGrass) or frontPoint.within(rightGrass):
			gpsConflict.data = 1
			pubGps.publish(gpsConflict)
			print "1"
			return
		
		point = findForwardPoint(gpsMsg, (gpsMsg.track + 45), 0.5)
		rightLat, rightLon = point.latitude, point.longitude
		rightPoint = Point(frontLat, frontLon)
		if frontPoint.within(leftGrass) or frontPoint.within(rightGrass):
			gpsConflict.data = 2
			pubGps.publish(gpsConflict)
			print "2"
			return
		
		point = findForwardPoint(gpsMsg, (gpsMsg.track + 315), 0.5)
		leftLat, leftLon = point.latitude, point.longitude
		leftPoint = Point(frontLat, frontLon)
		if frontPoint.within(leftGrass) or frontPoint.within(rightGrass):
			gpsConflict.data = 3
			pubGps.publish(gpsConflict)
			print "3"
			return
		
		# Since there are no conflicts, ensure that heading is going to destination
		bearing = findBearing(phi2, lambda2, phi1, lambda1)
		# Send "obstacle" to buggy since buggy is drifting left
		if (bearing >= 340):
			gpsConflict.data = 3
			pubGps.publish(gpsConflict)
		
		# Send "obstacle" to buggy since buggy is drifting right
		if (bearing <= 20):
			gpsConflict.data = 2
			pubGps.publish(gpsConflict)
		
		# Nothing to report if it reaches here
		gpsConflict.data = 0
		pubGps.publish(gpsConflict)
		
def findBearing(destLat, destLon, currentLat, currentLon):
	angle = degrees(atan2(destLon - currentLon, destLat - currentLat))
	bearing = (angle + 360) % 360
	return bearing

# phi is latitude, lambda is longitude, R is earth's radius
# pythogorean is accurate enough for very small distances
def pythagorean(lmbda, phi):
	R = 6378100
	phi1 = math.radians(phi)
	lambda1 = math.radians(lmbda)
	x = (lambda2 - lambda1) * math.cos((phi1+phi2) / 2);
	y = (phi2 - phi1);
	d = math.sqrt((x * x) + (y * y)) * R
	return d
	
def findForwardPoint(gpsMsg, bearing, d):
	# Distance in front of point (meters)
	origin = geopy.Point(gpsMsg.latitude, gpsMsg.longitude)
	forwardPoint = geodesic(meters=d).destination(origin, bearing)
	lat2, lon2 = forwardPoint.latitude, forwardPoint.longitude
	return forwardPoint

def init():
	rospy.init_node('gps_processor', anonymous=True)
	
	# GPS publisher
	# Sends 0 (Clear) 
	#				1 (Conflict on left of buggy)
	#				2 (Conflict on Front)
	# 			3 (Conflict on Right)
	global pubGps
	pubGps = rospy.Publisher('toggle_servo', Int16, queue_size = 1)

	# GPS publisher
	subGps = rospy.Subscriber('extended_fix', GPSFix, callback = gpsCallback, queue_size = 1)
	
if __name__ == '__main__':
	init()
	rospy.spin()
