##############################################################################
#                                                                            #
#     University of Massachusetts Dartmouth  ECE 565 - Operating Systems     #
#     Title: ROSConvoyNode0.py   Authors: Dylan Z. Baker, George Haddad      #
#     Description: This demo script facilitates communication for a robotic  #
#          vehicle convoy over a ROS bus. It also handles the actuation of   #
#          the driving motors for this vehicle.                              #
#     Creation Date: 5/11/2017                   Last Modified: 26/11/2017   #
#                                                                            #
##############################################################################

#!/usr/bin/env python

import thread
import time
import rospy
import math
import RPi.GPIO as GPIO
import time
from std_msgs.msg import String


# Publishes a formatted time-stamped message based on whether this node is moving,
#   what its velocity is, and what its heading (direction) is.
# Message format: <timestamp>|<moving (t/f)>|<velocity (m/s)>|<heading (rads, North=0)>
def PublishStatus(moving, velocity, heading):
	if moving:
		messageToPublish = "%s|1|%f|%f" % (rospy.get_time(), velocity, heading)
	else:
		messageToPublish = "%s|0|%f|%f" % (rospy.get_time(), velocity, heading)
	
	PublishMessage(messageToPublish, 'carconvoy0')


# Publishes a generic string message with the given message and topic.
def PublishMessage(message, topic):
	pub = rospy.Publisher(topic, String, queue_size=10)
	rospy.loginfo(message)
	pub.publish(message)


# Listens indefinitely for status messages on the bus. If it doesn't receive a message,
#   it will go into a safe (stopping) mode and notify the nodes in back until it begins
#   receiving status again.
def PublishStatusMessages():
	global lastReceivedStatus
	global currentVelocity
	global currentHeading
	
	GPIO.setmode(GPIO.BCM)

	#setup for ultrasonic sensor/motor
	TRIG = 23 
	ECHO = 24
	GPIO.setup(TRIG,GPIO.OUT)
	GPIO.setup(ECHO,GPIO.IN)
	GPIO.setup(27, GPIO.OUT)

	p = GPIO.PWM(27,100)#(motor pin,frequency)
	p.start(0)

	GPIO.output(TRIG,False)

	time.sleep(2)
	
	#car starts to move in this try statment
	try:
		while True: #begin ultrasonic sensor read
			GPIO.output(TRIG, True)
			time.sleep(0.00001)
			GPIO.output(TRIG, False)

			while GPIO.input(ECHO)==0:
				pulse_start = time.time()

			while GPIO.input(ECHO)==1:
				pulse_end = time.time()

			pulse_duration = pulse_end - pulse_start

			distance = pulse_duration * 17150

			distance = round(distance, 2) #distance in centimeters
			if distance <= 40: #if object infront of car is 40 cm or less away then go to slow stop
				# Not moving, speed of 0, heading of 0.
				PublishStatus(0, 0, 0)
				for i in range(100):#decelerate
					p.ChangeDutyCycle(100 - i)
					time.sleep(0.02)
				break
			else:
				# Is moving, speed of 1, heading of 0.
				PublishStatus(1, 1, 0)
				p.ChangeDutyCycle(100)#continue high speed
				
			print("Distance:",distance,"cm")#output distance

			time.sleep(.1)
		print("done")#car stopped
        
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()

	# Uncomment to test basic publishing functionality.
	#while True:
	#	time.sleep(0.05)	# Updates every 50 ms.
	#	PublishStatus(1, 9.8, 0)


# Handles Safe Mode. When in this mode, the car will safely come to a stop and notify all
#   cars behind it. If successful communication is resumed with the nodes in front of it,
#   Safe Mode will be exited.
def EnterSafeMode():
	global lastReceivedStatus
	# Entering safe mode. Send Safe Mode Message, along with
	#   status.
	PublishMessage("SAFEON", 'carconvoy0')
	while lastReceivedStatus == "unset":
		# Slow down.
		PublishStatus(0, currentVelocity, currentHeading)
		time.sleep(0.01)
	# Out of safe mode. Send Safe Mode Off Message. TODO: Will need to verify receipt.
	PublishMessage("SAFEOFF", 'carconvoy0')


# Processes message from ROS bus. If this is a safe mode message, it will handle that
#   accordingly. Otherwise, it will store the message in lastReceivedStatus, for the
#   listening thread to handle.
def ProcessMessage(data):
	global lastReceivedStatus
	if data == "SAFEON":
		# Handle safe mode.
		x = 1
	elif data == "SAFEOFF":
		# Handle exiting safe mode.
		x = 1
	else:
		lastReceivedStatus = data


# Initializes this ROS node with the given name.
def InitializeNode(name):
	rospy.init_node(name, anonymous=True)


# Main routine to start listening thread and publish status periodically if this is the
#   Primary car.
if __name__ == '__main__':
	try:
		# Last received status from the node most immediately in front of this one. This will
		#   be purged after 100 ms.
		global lastReceivedStatus
		lastReceivedStatus = "unset"
		# Current Velocity
		global currentVelocity
		currentVelocity = 0
		# Current Heading
		global currentHeading
		currentHeading = 0.00
		
		InitializeNode('convoynode0')
				
		thread.start_new_thread(PublishStatusMessages, ())
		
		rospy.Subscriber('carconvoy0', String, ProcessMessage)
		
		# Prevent Python from exiting.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
