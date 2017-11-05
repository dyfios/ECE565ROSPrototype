##############################################################################
#                                                                            #
#     University of Massachusetts Dartmouth  ECE 565 - Operating Systems     #
#     Title: ROSConvoy.py   Authors: Dylan Z. Baker,                         #
#     Description: This demo script facilitates communication for a robotic  #
#          vehicle convoy over a ROS bus. It also handles the actuation of   #
#          the driving motors for this vehicle.                              #
#     Creation Date: 5/11/2017                   Last Modified: 5/11/2017    #
#                                                                            #
##############################################################################

#!/usr/bin/env python

import thread
import time
import rospy
from std_msgs.msg import String


# Last received status from the node most immediately in front of this one. This will
#   be purged after 100 ms.
lastReceivedStatus = "unset"
# Current Velocity.
currentVelocity = 0
# Current Heading.
currentHeading = 0.00


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
def ListenForMessages():
	while True:
		time.sleep(0.05)	# Updates every 50 ms.
		lastMsgFields = lastReceivedStatus.split("|")
		if (float(lastMsgFields[0]) - rospy.get_time) > 0.1:
			lastReceivedStatus = "unset"
		
		if lastReceivedStatus == "unset":
			// Enter Safe Mode.
			EnterSafeMode()
		else:
			// Apply velocity to motors.


# Handles Safe Mode. When in this mode, the car will safely come to a stop and notify all
#   cars behind it. If successful communication is resumed with the nodes in front of it,
#   Safe Mode will be exited.
def EnterSafeMode():
	// Entering safe mode. Send Safe Mode Message, along with
	//   status.
	PublishMessage("SAFEON", 'carconvoy0')
	while lastReceivedStatus == "unset":
		// Slow down.
		PublishStatus(0, currentVelocity, currentHeading)
		time.sleep(0.01)
	// Out of safe mode. Send Safe Mode Off Message. TODO: Will need to verify receipt.
	PublishMessage("SAFEOFF", 'carconvoy0')


# Processes message from ROS bus. If this is a safe mode message, it will handle that
#   accordingly. Otherwise, it will store the message in lastReceivedStatus, for the
#   listening thread to handle.
def ProcessMessage(data):
	if data == "SAFEON":
		// Handle safe mode.
	elif data == SAFEOFF:
		// Handle exiting safe mode.
	else:
		lastReceivedStatus = data


# Initializes this ROS node with the given name.
def InitializeNode(name):
	rospy.init_node(name, anonymous=True)


# Main routine to start listening thread and publish status periodically if this is the
#   Primary car.
if __name__ == '__main__':
	try:
		InitializeNode('convoynode0')
		
		thread.start_new_thread(ListenForMessages)
		
		rospy.Subscriber('carconvoy0', String, ProcessMessage)
		
		# Prevent Python from exiting.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
