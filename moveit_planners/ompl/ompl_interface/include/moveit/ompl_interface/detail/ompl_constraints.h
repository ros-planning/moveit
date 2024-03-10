/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of KU Leuven nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jeroen De Maeyer, Boston Cleek */

#pragma once

#include <ostream>

#include <ompl/base/Constraint.h>

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/macros/class_forward.h>

namespace ompl_interface
{
MOVEIT_CLASS_FORWARD(BaseConstraint);
MOVEIT_CLASS_FORWARD(BoxConstraint);
MOVEIT_CLASS_FORWARD(OrientationConstraint);

/** \brief Represents upper and lower bound on the elements of a vector.
 *
 * OMPL ConstrainedStateSpace requires a model of the constraints given as:
 *  f1(joint_values) = 0
 *  f2(joint_values) = 0
 *  f3(joint_values) = 0
 *  ...
 *
 * So we use a penalty function to convert bounds to an equality constraint.
 * If you do need equality constraint, you can represent them by setting the upper bound and lower bound almost equal.
 * Or you can use the EqualityPositionConstraint version by setting the name of the constraint to
 * "use_equality_constraints". But the latter will ignore bounds on other dimensions.
 * **/
class Bounds
{
public:
  Bounds();
  Bounds(const std::vector<double>& lower, const std::vector<double>& upper);
  /** \brief Distance to region inside bounds
   *
   * Distance of a given value outside the bounds, zero inside the bounds.
   * Creates a penalty function that looks like this:
   *
   * (penalty) ^
   *           | \         /
   *           |  \       /
   *           |   \_____/
   *           |----------------> (variable to be constrained)
   * */
  Eigen::VectorXd penalty(const Eigen::Ref<const Eigen::VectorXd>& x) const;

  /** \brief Derivative of the penalty function
   * ^
   * |
   * | -1-1-1 0 0 0 +1+1+1
   * |------------------------>
   * **/
  Eigen::VectorXd derivative(const Eigen::Ref<const Eigen::VectorXd>& x) const;

  std::size_t size() const;

private:
  std::vector<double> lower_, upper_;
  std::size_t size_;

  friend std::ostream& operator<<(std::ostream& os, const ompl_interface::Bounds& bounds);
};

/** \brief Pretty printing of bounds. **/
std::ostream& operator<<(std::ostream& os, const ompl_interface::Bounds& bounds);

/****************************
 * Base class for constraints
 * **************************/
/** \brief Abstract base class for different types of constraints, implementations of ompl::base::Constraint
 *
 * To create a constrained state space in OMPL, we need a model of the constraints, that can be written in the form of
 * equality constraints F(joint_values) = 0. This class uses `Bounds` defined above, to convert:
 *
 *    lower_bound < scalar value < upper bound
 *
 * into an equation of the form f(x) = 0.
 *
 * The 'scalar value' can be the difference between the position or orientation of a link and a target position or
 * orientation, or any other error metric that can be calculated using the `moveit::core::RobotModel` and
 * `moveit::core::JointModelGroup`.
 * */
class BaseConstraint : public ompl::base::Constraint
{
public:
  /** \brief Construct a BaseConstraint using 3 `num_cons` by default because all constraints currently implemented have
   * 3 constraint equations. **/
  BaseConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                 const unsigned int num_dofs, const unsigned int num_cons = 3);

  /** \brief Initialize constraint based on message content.
   *
   * This is necessary because we cannot call the pure virtual
   * parseConstraintsMsg method from the constructor of this class.
   * */
  void init(const moveit_msgs::Constraints& constraints);

  /** OMPL's main constraint evaluation function.
   *
   *  OMPL requires you to override at least "function" which represents the constraint F(q) = 0
   * */
  void function(const Eigen::Ref<const Eigen::VectorXd>& joint_values, Eigen::Ref<Eigen::VectorXd> out) const override;

  /** \brief Jacobian of the constraint function.
   *
   * Optionally you can also provide dF(q)/dq, the Jacobian of  the constraint.
   *
   * */
  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values, Eigen::Ref<Eigen::MatrixXd> out) const override;

  /** \brief Wrapper for forward kinematics calculated by MoveIt's Robot State.
   *
   * TODO(jeroendm) Are these actually const, as the robot state is modified? How come it works?
   * Also, output arguments could be significantly more performant,
   * but MoveIt's robot state does not support passing Eigen::Ref objects at the moment.
   * */
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief Calculate the robot's geometric Jacobian using MoveIt's Robot State.
   *
   * Ideally I would pass the output argument from OMPL's jacobian function directly,
   * but I cannot pass an object of type , Eigen::Ref<Eigen::MatrixXd> to MoveIt's
   * Jacobian method.
   * */
  Eigen::MatrixXd robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief Parse bounds on position and orientation parameters from MoveIt's constraint message.
   *
   * This can be non-trivial given the often complex structure of these messages.
   * */
  virtual void parseConstraintMsg(const moveit_msgs::Constraints& constraints) = 0;

  /** \brief For inequality constraints: calculate the value of the parameter that is being constrained by the bounds.
   *
   * In this Position constraints case, it calculates the x, y and z position
   * of the end-effector. This error is then converted in generic equality constraints in the implementation of
   * `ompl_interface::BaseConstraint::function`.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::function directly and ignore
   * the bounds calculation.
   * */
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& /*x*/) const;

  /** \brief For inequality constraints: calculate the Jacobian for the current parameters that are being constrained.
   *   *
   * This error jacobian, as the name suggests, is only the jacobian of the position / orientation / ... error.
   * It does not take into account the derivative of the penalty functions defined in the Bounds class.
   * This correction is added in the implementation of of BaseConstraint::jacobian.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::jacobian directly and ignore
   * the bounds calculation.
   *
   * TODO(jeroendm), Maybe also use an output argument as in `ompl::base::Constraint::jacobian(x, out)` for better
   * performance?
   * */
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& /*x*/) const;

  // the methods below are specifically for debugging and testing

  const std::string& getLinkName()
  {
    return link_name_;
  }

  const Eigen::Vector3d getTargetPosition()
  {
    return target_position_;
  }

  const Eigen::Quaterniond getTargetOrientation()
  {
    return target_orientation_;
  }

protected:
  /** \brief Thread-safe storage of the robot state.
   *
   * The robot state is modified for kinematic calculations. As an instance of this class is possibly used in multiple
   * threads due to OMPL's LazyGoalSampler, we need a separate robot state in every thread.
   * */
  TSStateStorage state_storage_;
  const robot_state::JointModelGroup* joint_model_group_;

  // all attributes below can be considered const as soon as the constraint message is parsed
  // but I (jeroendm) do not know how to elegantly express this in C++
  // parsing the constraints message and passing all this data members separately to the constructor
  // is a solution, but it adds complexity outside this class, which is also not ideal.

  /** \brief Robot link the constraints are applied to. */
  std::string link_name_;

  /** \brief Upper and lower bounds on constrained variables. */
  Bounds bounds_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Vector3d target_position_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Quaterniond target_orientation_;

public:
  // Macro for classes containing fixed size eigen vectors that are dynamically allocated when used.
  // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/******************************************
 * Box constraint
 * ****************************************/
/** \brief Box shaped position constraints
 *
 * Reads bounds on x, y and z position from a position constraint
 * at constraint_region.primitives[0].dimensions.
 * Where the primitive has to be of type `shape_msgs/SolidPrimitive.BOX`.
 *
 * These bounds are applied around the nominal position and orientation
 * of the box.
 * */
class BoxConstraint : public BaseConstraint
{
public:
  BoxConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                const unsigned int num_dofs);

  /** \brief Parse bounds on position parameters from MoveIt's constraint message.
   *
   * This can be non-trivial given the often complex structure of these messages.
   * */
  void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;

  /** \brief For inequality constraints: calculate the value of the parameter that is being constrained by the bounds.
   *
   * In this Position constraints case, it calculates the x, y and z position
   * of the end-effector. This error is then converted in generic equality constraints in the implementation of
   * `ompl_interface::BaseConstraint::function`.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::function directly and ignore
   * the bounds calculation.
   * */
  Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

  /** \brief For inequality constraints: calculate the Jacobian for the current parameters that are being constrained.
   *   *
   * This error jacobian, as the name suggests, is only the jacobian of the position / orientation / ... error.
   * It does not take into account the derivative of the penalty functions defined in the Bounds class.
   * This correction is added in the implementation of of BaseConstraint::jacobian.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::jacobian directly and ignore
   * the bounds calculation.
   *
   * TODO(jeroendm), Maybe also use an output argument as in `ompl::base::Constraint::jacobian(x, out)` for better
   * performance?
   * */
  Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

/******************************************
 * Equality Position Constraints
 * ****************************************/
/** \brief Equality constraints on a link's position.
 *
 *  When you set the name of a constraint to 'use_equality_constraints', all constraints with a dimension lower than
 * `EQUALITY_CONSTRAINT_THRESHOLD` will be modelled as equality constraints.
 *
 * The dimension value for the others are ignored. For example, a box with dimensions [1.0, 1e-5, 1.0]
 * will result in equality constraints on the y-position, and no constraints on the x or z-position.
 *
 * TODO(jeroendm) We could make this a base class `EqualityConstraints` with a specialization for position and orientation
 * constraints in the future. But the direct overriding of `function` and `jacobian` is probably more performant.
 * */
class EqualityPositionConstraint : public BaseConstraint
{
public:
  EqualityPositionConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                             const unsigned int num_dofs);

  /** \brief Parse bounds on position parameters from MoveIt's constraint message.
   *
   * This can be non-trivial given the often complex structure of these messages.
   * */
  void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;

  void function(const Eigen::Ref<const Eigen::VectorXd>& joint_values, Eigen::Ref<Eigen::VectorXd> out) const override;

  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values, Eigen::Ref<Eigen::MatrixXd> out) const override;

private:
  /** \brief Position bounds under this threshold are interpreted as equality constraints, the others as unbounded.
   *
   * WARNING: Below follows the explanation of an ugly hack. Ideally, the user could specify equality constraints by
   * setting the constraint dimension to zero. However, this would result in a constraint region primitive with a zero
   * dimension in MoveIt, which the planner can (almost) never satisfy. Therefore we use a threshold value, below which
   * the constraint is interpreted as an equality constraint. This threshold value is not used in the planning algorithm itself!
   *
   * This threshold value should be larger than the tolerance of the constraints specified in OMPL
   * (ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE = 1e-4).
   *
   * This is necessary because the constraints are also checked by MoveIt in the StateValidity checker. If this check
   * would use a stricter tolerance than was used to satisfy the constraints in OMPL, all states would be invalid.
   * Therefore the dimension of an equality constraint specified in the constraint message should be at least equal the
   * the tolerance used by OMPL to project onto the constraints. To avoid checking for exact equality on floats, the
   * threshold is made a bit larger.
   *
   * equality_constraint_threshold_ > tolerance in constraint message > OMPL projection tolerance
   *
   * That's why the value is 1e-3 > 1e-4.
   * Now the user can specify any value between 1e-3 and 1e-4 to specify an equality constraint.
   * **/
  static constexpr double EQUALITY_CONSTRAINT_THRESHOLD{ 0.001 };

  /** \brief Bool vector indicating which dimensions are constrained. **/
  std::vector<bool> is_dim_constrained_;
};

/******************************************
 * Orientation constraints
 * ****************************************/
/** \brief Orientation constraints parameterized using exponential coordinates.
 *
 * An orientation constraint is modeled as a deviation from a target orientation.
 * The deviation is represented using exponential coordinates. A three element vector represents the rotation axis
 * multiplied with the angle in radians around this axis.
 *
 *  R_error = R_end_effector ^ (-1) * R_target
 *  R_error -> rotation angle and axis           (using Eigen3)
 *  error = angle * axis                         (vector with three elements)
 *
 *  And then the constraints can be written as
 *
 *     - absolute_x_axis_tolerance < error[0] < absolute_x_axis_tolerance
 *     - absolute_y_axis_tolerance < error[1] < absolute_y_axis_tolerance
 *     - absolute_z_axis_tolerance < error[2] < absolute_z_axis_tolerance
 *
 * **IMPORTANT** It is NOT how orientation error is handled in the default MoveIt constraint samplers, where XYZ
 * intrinsic euler angles are used. Using exponential coordinates is analog to how orientation error is calculated in
 * the TrajOpt  motion planner.
 *
 * */
class OrientationConstraint : public BaseConstraint
{
public:
  OrientationConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                        const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }

  /** \brief Parse bounds on orientation parameters from MoveIt's constraint message.
   *
   * This can be non-trivial given the often complex structure of these messages.
   * */
  void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;

  /** \brief For inequality constraints: calculate the value of the parameter that is being constrained by the bounds.
   *
   * In this orientation constraints case, it calculates the orientation
   * of the end-effector. This error is then converted in generic equality constraints in the implementation of
   * `ompl_interface::BaseConstraint::function`.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::function directly and ignore
   * the bounds calculation.
   * */
  Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

  /** \brief For inequality constraints: calculate the Jacobian for the current parameters that are being constrained.
   *   *
   * This error jacobian, as the name suggests, is only the jacobian of the position / orientation / ... error.
   * It does not take into account the derivative of the penalty functions defined in the Bounds class.
   * This correction is added in the implementation of of BaseConstraint::jacobian.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::jacobian directly and ignore
   * the bounds calculation.
   *
   * TODO(jeroendm), Maybe also use an output argument as in `ompl::base::Constraint::jacobian(x, out)` for better
   * performance?
   * */
  Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

/** \brief Extract position constraints from the MoveIt message.
 *
 * Assumes there is a single primitive of type `shape_msgs/SolidPrimitive.BOX`.
 * The dimensions of the box are the bounds on the deviation of the link origin from
 * the target pose, given in constraint_regions.primitive_poses[0].
 * */
Bounds positionConstraintMsgToBoundVector(const moveit_msgs::PositionConstraint& pos_con);

/** \brief Extract orientation constraints from the MoveIt message
 *
 * These bounds are assumed to be centered around the target orientation / desired orientation
 * given in the "orientation" field in the message.
 * These bounds represent orientation error between the desired orientation and the current orientation of the
 * end-effector.
 *
 * The "absolute_x_axis_tolerance", "absolute_y_axis_tolerance" and "absolute_z_axis_tolerance" are interpreted as
 * the width of the tolerance regions around the target orientation, represented using exponential coordinates.
 *
 * */
Bounds orientationConstraintMsgToBoundVector(const moveit_msgs::OrientationConstraint& ori_con);

/** \brief Factory to create constraints based on what is in the MoveIt constraint message. **/
std::shared_ptr<BaseConstraint> createOMPLConstraints(const robot_model::RobotModelConstPtr& robot_model,
                                                      const std::string& group,
                                                      const moveit_msgs::Constraints& constraints);

/** \brief  Return a matrix to convert angular velocity to angle-axis velocity
 *  Based on:
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
 * Eq. 2.107
 * */
inline Eigen::Matrix3d angularVelocityToAngleAxis(const double& angle, const Eigen::Ref<const Eigen::Vector3d>& axis)
{
  double t{ std::abs(angle) };
  Eigen::Matrix3d r_skew;
  r_skew << 0, -axis[2], axis[1], axis[2], 0, -axis[0], -axis[1], axis[0], 0;
  r_skew *= angle;

  double c;
  c = (1 - 0.5 * t * std::sin(t) / (1 - std::cos(t)));

  return Eigen::Matrix3d::Identity() - 0.5 * r_skew + (r_skew * r_skew / (t * t)) * c;
}

}  // namespace ompl_interface
