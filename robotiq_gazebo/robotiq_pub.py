#!/usr/bin/env python

#    Example ROS node. Demonstrates how to write a ROS node to control the simulated WAM.
#    Copyright (C) 2013  Benjamin Blumer
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
This script is used to publish a trajectory on the ROS topic joint_traj.

The included WAM plugin subscribes to this topic, and carries out the motion.
The trajectory starts every joint off at a value of .1 and increases the
value by .01 rads every .1 seconds. You can easily replace this with a 
meaningful trajectory by filling one_set_of_joint_coordinates with the
coordinates you want before appending it to all_sets_of_joint_coordinates.

I've heavily commented the code to make it accessible for people who are new
to ROS. In doing so, I've violated the good practice of "Assume the
reader knows Python better than you". I've done so in the hope that it will
help people get their own projects up and running quickly.  If you have any
recomendations on making this more accessible, please let me know
via my github account or make the change yourself and request a "pull".
"""
import roslib; roslib.load_manifest('robotiq_s_model_articulated_msgs')
import rospy; 
from std_msgs.msg import Float64
from robotiq_s_model_articulated_msgs.msg import _SModelRobotOutput  as outputMsg

#import _Num.py

def genCommand(command):
	command.rACT = 1
	command.rMOD = 0
	command.rGTO = 1
	command.rATR = 0
	command.rICF = 1
	command.rICS = 1
	command.rPRA = 0
	command.rSPA = 255
	command.rFRA = 50
	command.rPRB = 0
	command.rSPB = 255
	command.rFRB = 50
	command.rPRC = 0
	command.rSPC = 255
	command.rFRC = 50
	command.rPRS = 0
	command.rSPS = 50
	command.rFRS= 50

	val = raw_input('\nPosition A: ')
	command.rPRA = int(val)
	val = raw_input('\nPosition B: ')
	command.rPRB = int(val)
	val = raw_input('\nPosition C: ')
	command.rPRC = int(val)
	val = raw_input('\nPosition S: ')
	command.rPRS = int(val)
	return command

def controller():
	"""Main loop which requests new commands and publish them on the SModelRobotOutput topic."""

	pub = rospy.Publisher('/left_hand/command', outputMsg.SModelRobotOutput, queue_size=10)
	# This node is called trajectory_giver
	rospy.init_node('hand_pub')
	command = outputMsg.SModelRobotOutput();
	command = genCommand(command)

	pub.publish(command)
	str = "Published a joint trajectory as time %s" % rospy.get_time()
	rospy.loginfo(str)
	rospy.sleep(0.1)

if __name__ == '__main__':
	while not rospy.is_shutdown():
		controller()
