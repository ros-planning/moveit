#!/usr/bin/env python3
import moveit_commander
import rospy
import tf2_ros
from controller_manager_msgs.srv import (SwitchController,
                                         SwitchControllerRequest)
from geometry_msgs.msg import (Point, Quaternion, TransformStamped, Pose,
                               TwistStamped, Vector3)
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

TF_CONTROL = "wrist_3_link"  # Frame used for transformation
UPDATE_RATE = 50  # Update rate for servoing [Hz]
STICKOUT = 0.3
PUBLISH_TF = True


class RobotController():

    ################## INITIALIZATION ##################

    def __init__(self, sim: Bool, mode: int):

        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(40))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.group = moveit_commander.MoveGroupCommander("welding_endeffector")

        self.setStickoutTransform(0, 0, STICKOUT, 0, 90, 0)
        # Initializing
        self.initServices()
        self.changeController(0)

        self.pub_twist = rospy.Publisher("/servoTwist",
                                         TwistStamped,
                                         queue_size=1)
        rospy.sleep(1)

    def initServices(self):
        # Service for changing controller
        name = 'controller_manager/switch_controller'

        rospy.wait_for_service(name, rospy.Duration(25))
        self.srv_switchController = rospy.ServiceProxy(name, SwitchController)

    ################## STICKOUT COMPUTATIONS ##################

    def setStickoutTransform(self, x: float, y: float, z: float, rx: float,
                             ry: float, rz: float) -> None:
        """Sets transform for stickout in tf tree and saves in property

        Args:
            x (float): Offset in X [m]
            y (float): Offset in Y [m]
            z (float): Offset in Z [m]
            rx (float): Rotation around X [deg]
            ry (float): Rotation around Y [deg]
            rz (float): Rotation around Z [deg]
        """
        if PUBLISH_TF:
            tf_stickout_to_tcp = TransformStamped()
            tf_stickout_to_tcp.header.stamp = rospy.Time.now()

            tf_stickout_to_tcp.header.frame_id = TF_CONTROL
            tf_stickout_to_tcp.child_frame_id = "TCP_M"

            tf_stickout_to_tcp.transform.translation = Point(x, y, z)
            rot_quat = Rotation.from_euler('XYZ', [rx, ry, rz],
                                           degrees=True).as_quat()
            rospy.loginfo(rot_quat)
            tf_stickout_to_tcp.transform.rotation = Quaternion(*rot_quat)
            self.tf_broadcaster.sendTransform(tf_stickout_to_tcp)

        # Add collision object
        # self.scene.clear()
        colObj = CollisionObject()
        colObj.id = "welding_frames"
        colObj.header.frame_id = "wrist_3_link"
        colObj.header.stamp = rospy.Time.now()
        colObj.pose = Pose(orientation=Quaternion(0, 0, 0, 1))

        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation = Quaternion(*rot_quat)

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        xyz_dim = 0.003
        box.dimensions = [xyz_dim, xyz_dim, xyz_dim]

        colObj.primitives = [box]
        colObj.primitive_poses = [pose]
        colObj.subframe_names = ['tcf']
        colObj.subframe_poses = [pose]
        colObj.operation = CollisionObject.ADD

        rospy.loginfo("ColObj: {}".format(colObj))
        rospy.loginfo(self.scene.get_known_object_names())
        # Object needs to be added, otherwise scene somehow doesn't update
        self.scene.add_object(colObj)
        # Attach object (only for visualization)
        aco = AttachedCollisionObject()
        aco.link_name = 'wrist_3_link'
        aco.object = colObj
        aco.touch_links = self.robot.get_link_names(group="welding_endeffector")
        self.scene.attach_object(aco)

    def changeController(self, mode: int):
        """Change Controller: mode==0 -> MoveIt, mode==1 -> ServoArm

        Args:
            mode (int): mode
        """
        control_servo = ['joint_group_position_controller']
        control_manual = ['pos_joint_traj_controller']
        msg = SwitchControllerRequest()
        msg.strictness = 2
        if mode == 0:  # MoveIt Control
            msg.start_controllers = control_manual
            msg.stop_controllers = control_servo
        elif mode == 1:  # Servo Control
            msg.start_controllers = control_servo
            msg.stop_controllers = control_manual
            rospy.sleep(0.3)
        elif mode == 2:  # Moveit control Real Robot - start
            msg.start_controllers = ["scaled_pos_joint_traj_controller"]
            msg.stop_controllers = control_manual
            rospy.sleep(0.3)
        elif mode == 3:  # Moveit control Real Robot - end
            msg.start_controllers = control_manual
            msg.stop_controllers = ["scaled_pos_joint_traj_controller"]
            rospy.sleep(0.3)
        else:
            return
        self.srv_switchController(msg)

    def testMotion(self):

        # Change controller to servo
        self.changeController(1)
        # test servo
        ur = rospy.Rate(5)
        motions = [
            [0, 0, 0, 0, 0.5, 0],
            [0, 0, 0, 0, 0, 0.5],
            [0, 0, 0, 0.5, 0, 0],
            [0.1, 0, 0, 0, 0, 0],
            [0, 0.1, 0, 0, 0, 0],
            [0, 0, 0.1, 0, 0, 0],
        ]

        for motion in motions:
            i = 0
            while not rospy.is_shutdown() and i < 25:
                i += 1

                # frame = "TCP_M"
                frame = "welding_frames/tcf"
                servo_msg = TwistStamped()
                servo_msg.header.stamp = rospy.Time.now()
                servo_msg.header.frame_id = frame
                servo_msg.twist.linear = Vector3(*motion[:3])
                servo_msg.twist.angular = Vector3(*motion[3:])

                rospy.loginfo([servo_msg.header.frame_id, motion])
                self.pub_twist.publish(servo_msg)
                ur.sleep()
        # Change controller back
        self.changeController(0)


if __name__ == "__main__":
    rospy.init_node("robot_controller")
    # rospy.sleep(3)
    sim = True
    mode = 1
    robot_controller = RobotController(sim, mode)
    robot_controller.changeController(1)
    robot_controller.testMotion()
    rospy.spin()
    rospy.loginfo('RobotController: Finished')
    robot_controller.changeController(0)  # Stop servo
    robot_controller.changeController(3)  # Stop trajectory controller
