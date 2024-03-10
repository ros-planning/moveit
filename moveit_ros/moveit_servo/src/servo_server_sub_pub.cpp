#include <moveit_servo/servo_server_sub_pub.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>  // For testing only
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <string>
#include <moveit_servo/make_shared_from_pool.h>

namespace moveit_servo
{
//----------------------------------------------------------------------
// CONSTRUCTORS
//----------------------------------------------------------------------

/** Construct a new FollowTaskFrame object */
ServoRobot::ServoRobot(ros::NodeHandle* nodehandle,
                       planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh(*nodehandle), servo(*nodehandle, planning_scene_monitor)
{
  servo.start();
  initializePublishers();
}

//----------------------------------------------------------------------
// INITIALIZING
//----------------------------------------------------------------------

/** Initialize Subscribers */
void ServoRobot::initializePublishers()
{
  ROS_INFO("ServoRobot: Initialize Publishers");
  servo_pub = nh.advertise<geometry_msgs::TwistStamped>(servo.getParameters().cartesian_command_in_topic, 1);
}

//----------------------------------------------------------------------
// CALLBACKS
//----------------------------------------------------------------------
void ServoRobot::servoing(geometry_msgs::TwistStamped servo_msg_in)
{
  // Create servo message
  auto servo_msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();

  // Fill in data in servo message
  servo_msg->twist = servo_msg_in.twist;
  servo_msg->header.frame_id = servo_msg_in.header.frame_id;
  servo_msg->header.stamp = ros::Time::now();
  servo_pub.publish(servo_msg);
}
}  // namespace moveit_servo

//----------------------------------------------------------------------
// MAIN FUNCTION
//----------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_robot");  // node name

  ROS_INFO("Start Follow Task Frame");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Load the planning scene monitor, HAS to be AFTER nodehandle initialization
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM("Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  moveit_servo::ServoRobot servoRobot(&nh, planning_scene_monitor);
  ros::Subscriber sub = nh.subscribe("/servoTwist", 1, &moveit_servo::ServoRobot::servoing, &servoRobot);
  ros::waitForShutdown();
  return 0;
}
