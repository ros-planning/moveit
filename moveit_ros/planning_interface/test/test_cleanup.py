#!/usr/bin/env python

import rospy
from moveit.planning_interface import MoveGroupInterface

group = MoveGroupInterface("manipulator", "robot_description", rospy.get_namespace())
