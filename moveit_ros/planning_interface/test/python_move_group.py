#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Bielefeld University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Robert Haschke

import unittest
import numpy as np
import rospy
import rostest
import sys
from rosgraph.names import ns_join

from moveit.planning_interface import MoveGroupInterface
from moveit.planning_interface import PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes


class MoveGroupInterfaceTest(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        self.group = MoveGroupInterface(
            self.PLANNING_GROUP, ns_join(PLANNING_NS, "robot_description"), PLANNING_NS
        )

    @classmethod
    def tearDown(self):
        pass

    def check_target_setting(self, expect, *args):
        if len(args) == 0:
            args = [expect]
        self.group.set_joint_value_target(*args)
        res = self.group.get_joint_value_target()
        self.assertTrue(
            np.all(np.asarray(res) == np.asarray(expect)),
            "Setting failed for %s, values: %s" % (type(args[0]), res),
        )

    def test_target_setting(self):
        n = self.group.get_variable_count()
        self.check_target_setting([0.1] * n)
        self.check_target_setting((0.2,) * n)
        self.check_target_setting(np.zeros(n))
        self.check_target_setting(
            [0.3] * n, {name: 0.3 for name in self.group.get_active_joints()}
        )
        self.check_target_setting([0.5] + [0.3] * (n - 1), "joint_1", 0.5)

    def plan(self, target):
        self.group.set_joint_value_target(target)
        return self.group.plan()

    def test_validation(self):
        current = np.asarray(self.group.get_current_joint_values())

        error_code1, plan1, time = self.plan(current + 0.2)
        error_code2, plan2, time = self.plan(current + 0.2)

        # both plans should have succeeded:
        self.assertEqual(error_code1.val, MoveItErrorCodes.SUCCESS)
        self.assertEqual(error_code2.val, MoveItErrorCodes.SUCCESS)

        # first plan should execute
        self.assertTrue(self.group.execute(plan1))

        # second plan should be invalid now (due to modified start point) and rejected
        self.assertFalse(self.group.execute(plan2))

        # newly planned trajectory should execute again
        error_code3, plan3, time = self.plan(current)
        self.assertEqual(error_code3.val, MoveItErrorCodes.SUCCESS)
        self.assertTrue(self.group.execute(plan3))

    def test_get_jacobian_matrix(self):
        current = self.group.get_current_joint_values()
        result = self.group.get_jacobian_matrix(current)
        # Value check by known value at the initial pose
        expected = np.array(
            [
                [0.0, 0.8, -0.2, 0.0, 0.0, 0.0],
                [0.89, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, -0.74, 0.74, 0.0, 0.1, 0.0],
                [0.0, 0.0, 0.0, -1.0, 0.0, -1.0],
                [0.0, 1.0, -1.0, 0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )
        self.assertTrue(np.allclose(result, expected))

        result = self.group.get_jacobian_matrix(current, [1.0, 1.0, 1.0])
        expected = np.array(
            [
                [1.0, 1.8, -1.2, 0.0, -1.0, 0.0],
                [1.89, 0.0, 0.0, 1.0, 0.0, 1.0],
                [0.0, -1.74, 1.74, 1.0, 1.1, 1.0],
                [0.0, 0.0, 0.0, -1.0, 0.0, -1.0],
                [0.0, 1.0, -1.0, 0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )


class PlanningSceneInterfaceTest(unittest.TestCase):
    def test_namespace(self):
        scene = PlanningSceneInterface()
        self.assertEqual(
            scene._pub_co.resolved_name, ns_join(PLANNING_NS, "collision_object")
        )
        self.assertEqual(
            scene._pub_aco.resolved_name,
            ns_join(PLANNING_NS, "attached_collision_object"),
        )

    def test_namespace_synchronous(self):
        scene = PlanningSceneInterface(synchronous=True)
        self.assertEqual(
            scene._apply_planning_scene_diff.resolved_name,
            ns_join(PLANNING_NS, "apply_planning_scene"),
        )


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_python_move_group"
    rospy.init_node(NODENAME)
    PLANNING_NS = rospy.get_param("~PLANNING_NS", "")
    rostest.rosrun(PKGNAME, NODENAME, MoveGroupInterfaceTest)
