#ifndef SERVO_ROBOT_SUB_PUB_H_
#define SERVO_ROBOT_SUB_PUB_H_
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>

namespace moveit_servo
{
class ServoRobot
{
public:
  ServoRobot(ros::NodeHandle* nodehandle, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
  void servoing(geometry_msgs::TwistStamped);

private:
  ros::NodeHandle nh;
  ros::Publisher servo_pub;

  moveit_servo::Servo servo;
  // Initializers
  void initializePublishers();
};

}  // end namespace moveit_servo

#endif