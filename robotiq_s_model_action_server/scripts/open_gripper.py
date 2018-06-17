#!/usr/bin/env python

import roslib
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output as outputMsg


def publisher():
    rospy.init_node('CloseGripper')
    pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=1)

    command = outputMsg.SModel_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150
    command.rPRA = 0

    rospy.sleep(1)
    pub.publish(command)

if __name__ == '__main__':
    publisher()
