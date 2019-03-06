#!/usr/bin/env python
import time

import pytest
import rospy
from geometry_msgs.msg import TwistStamped

from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory

JOG_ARM_STALE_TIMEOUT_S = 2.0
JOG_ARM_SETTLE_TIME_S = 3
ROS_SETTLE_TIME_S = 0.5

JOINT_JOG_COMMAND_TOPIC = 'jog_arm_server/joint_delta_jog_cmds'
CARTESIAN_JOG_COMMAND_TOPIC = 'jog_arm_server/delta_jog_cmds'

COMMAND_OUT_TOPIC = 'jog_arm_server/command'


@pytest.fixture
def node():
    return rospy.init_node('pytest', anonymous=True)


class JointJogCmd(object):
    def __init__(self):
        self._pub = rospy.Publisher(JOINT_JOG_COMMAND_TOPIC, JointJog, queue_size=1)

    def send_cmd(self, joint_pos):
        jj = JointJog()
        jj.header.stamp = rospy.Time.now()
        jj.joint_names = ['joint_{}'.format(i) for i in range(len(joint_pos))]
        jj.deltas = list(map(float, joint_pos))
        self._pub.publish(jj)


class CartesianJogCmd(object):
    def __init__(self):
        self._pub = rospy.Publisher(
            CARTESIAN_JOG_COMMAND_TOPIC, TwistStamped, queue_size=1
        )

    def send_cmd(self, linear, angular):
        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        ts.twist.linear.x, ts.twist.linear.y, ts.twist.linear.z = linear
        ts.twist.angular.x, ts.twist.angular.y, ts.twist.angular.z = angular
        self._pub.publish(ts)

def test_jog_arm_generates_joint_trajectory_when_joint_jog_command_is_received(node):
    sub = rospy.Subscriber(
        COMMAND_OUT_TOPIC, JointTrajectory, lambda x: received.append(x)
    )
    joint_cmd = JointJogCmd()
    cartesian_cmd = CartesianJogCmd()
    time.sleep(ROS_SETTLE_TIME_S)  # wait for pub/subs to settle
    time.sleep(JOG_ARM_SETTLE_TIME_S)  # wait for jog_arm server to init
    # This zero-command should produce no output
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 0])
    received = []
    rospy.sleep(1)
    assert len(received) <= 1

    # This nonzero command should produce jogging output
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 1])
    received = []
    rospy.sleep(1)
    assert len(received) > 1
