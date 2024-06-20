#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_interface/collision_terms.h>
//#include <trajopt/basic_types.h>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/expr_vec_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/world.h>

namespace trajopt_interface
{
// converts a vector of contacts to a vector of distances
void CollisionsToDistances(const trajopt_interface::ContactResultVector& dist_results, DblVec& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());

  // how to get distance from MoveIt to feed dists
  for (auto i = 0u; i < dist_results.size(); ++i)
    dists.push_back(dist_results[i].distance);

  // for (auto i = 0u; i < dist_results.size(); ++i)
  //   dists.push_back(dist_results[i].depth);
}

// directly related to equation 16 in the TrajOpt paper (the version I have)
// this function goes over all the contact in dist_results and calcualte the jac of the first-link-in-contact
// at the nearest point. Also it calculates the jac of the second-link-in-contact at the nearest point
// However, if the collision type is set to CCType_Between, then the cc_nearest_point is used only for the
// second link's jacobian calculations.
void CollisionsToDistanceExpressions(const trajopt_interface::ContactResultVector& dist_results,
                                     planning_scene::PlanningSceneConstPtr planning_scene, std::string planning_group,
                                     const sco::VarVector& vars, const DblVec& x, sco::AffExprVector& exprs,
                                     bool isTimestep1)
{
  // we are trying to fill up exprs which should be the signed distance expression ???
  // typedef std::vector<AffExpr> AffExprVector
  // AffExpr is struct in solver_interface.h:
  //        constant, coeffs, vars

  // typedef std::vector<Var> VarVector
  // typedef std:vector<double> DblVec

  Eigen::VectorXd dofvals = sco::getVec(x, vars);  // joint values
  // const std::vector<std::string>& link_names = manip->getLinkNames();
  const std::vector<std::string>& link_names = planning_scene->getRobotModel()->getLinkModelNames();

  // All collision data is in world corrdinate system. This provides the
  // transfrom for converting data between world frame and manipulator
  // frame.
  // tesseract::EnvStateConstPtr state = env->getState();
  // Eigen::Isometry3d change_base = state->transforms.at(manip->getBaseLinkName());
  // assert(change_base.isApprox(env->getState(manip->getJointNames(), dofvals)->transforms.at(manip->getBaseLinkName())));

  const moveit::core::RobotState robot_state = planning_scene->getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(planning_group);
  std::vector<std::string> group_joint_names = joint_model_group->getActiveJointModelNames();
  // trajopt::printVector("===>>> group joint names", group_joint_names);
  int group_dof = (int)group_joint_names.size();

  exprs.clear();
  exprs.reserve(dist_results.size());
  for (auto i = 0u; i < dist_results.size(); ++i)
  {
    // => for each ContactResult in the total ContactResultVector that contains all the contacts of each pair and all
    // the pairs const tesseract::ContactResult& res = dist_results[i];
    const trajopt_interface::ContactResult& res = dist_results[i];

    // cc_type is a member of ContactResult in tesseract and by defualt (in clear() function) is set
    // to cc_type = ContinouseCollisionType::CCType_None; I do the same here:
    ContinouseCollisionType cc_type = ContinouseCollisionType::CCType_None;

    // ContactResult in the original trajopt has two bodies and a distance
    // In MoveIt, we have Contact (corresponding to ContactResult in tesseract) type
    // which has depth (penetration between bodies). I am going to use this depth
    sco::AffExpr dist(res.distance);
    // sco::AffExpr dist(res.depth); // depth could be positive or negative, if I use bullet
    // is that a problem here?

    Eigen::VectorXd dist_grad_a, dist_grad_b;
    // => find the name of linkA in collision
    std::vector<std::string>::const_iterator itA = std::find(link_names.begin(), link_names.end(), res.link_names[0]);
    if (itA != link_names.end())
    {
      // => create a jacobian matrix
      Eigen::MatrixXd jac;
      jac.resize(6, group_dof);

      const moveit::core::LinkModel* link_0 = robot_state.getLinkModel(res.link_names[0]);

      /* Compute the Jacobian with reference to a particular point on a given link, for a specified group.
       *  - group: The group to compute the Jacobian for
       *  - link_name: The name of the link
       *  - reference_point_position: The reference point position (with respect to the link specified in link_name)
       *  - jacobian: The resultant jacobian
       *  - use_quaternion_representation Flag indicating if the Jacobian should use a quaternion representation
       * (default is false)
       *  - return True if jacobian was successfully computed, false otherwise
       */
      bool succeed = robot_state.getJacobian(joint_model_group, link_0, res.nearest_points[0], jac);
      // joint values are fixed, we are chaning links in contacts to calculate the jacobian

      // what does it meant to caluclate a jacobian at a given link and point ????????

      /** tesseract:
       *  Calculated jacobian at a link given joint angles
       *  jac: jacobian Output jacobian for a given link
       *  change_base: The transform from the base frame of the manipulator to the desired frame.
       *  joint_angles: Input vector of joint angles
       *  link_name: Name of link to calculate jacobian
       *  state: The state of the environment
       *  link_point: Point in the base frame for which to calculate the jacobian about
       *  output is a bool: True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
       */
      // manip->calcJacobian(jac,
      //                     change_base,
      //                     dofvals,
      //                     res.link_names[0],
      //                     *state,
      //                     res.nearest_points[0]);

      dist_grad_a = -res.normal.transpose() * jac.topRows(3);
      // this means: [normal . jac(1:3,1), normal . jac(1:3,2), normal . jac(1:3,3)]
      // in the paper: nT * J_PA

      sco::exprInc(dist, sco::varDot(dist_grad_a, vars));
      // varDot: creates an AffExpr with constant = 0, vars = second arg and coeffs = first arg
      // exprInc: adds the second arg to the first one

      sco::exprInc(dist, -dist_grad_a.dot(dofvals));
      // the above means: -dist_grad_a (dot_product) dofvals
    }

    std::vector<std::string>::const_iterator itB = std::find(link_names.begin(), link_names.end(), res.link_names[1]);
    if (itB != link_names.end())
    {
      Eigen::MatrixXd jac;
      jac.resize(6, group_dof);

      const moveit::core::LinkModel* link_1 = robot_state.getLinkModel(res.link_names[1]);

      // how do I calculate cc_nearest_points, Contact in MoveIt does not have such a thing
      bool succeed = robot_state.getJacobian(joint_model_group, link_1,
                                             (isTimestep1 &&
                                              (cc_type == trajopt_interface::ContinouseCollisionType::CCType_Between)) ?
                                                 res.cc_nearest_points[1] :
                                                 res.nearest_points[1],
                                             jac);

      dist_grad_b = res.normal.transpose() * jac.topRows(3);
      sco::exprInc(dist, sco::varDot(dist_grad_b, vars));
      sco::exprInc(dist, -dist_grad_b.dot(dofvals));
    }
    // DebugPrintInfo(res, dist_grad_a, dist_grad_b, dofvals, i == 0);

    if (itA != link_names.end() || itB != link_names.end())
    {
      exprs.push_back(dist);
    }
  }
}

// This function creates affine expression
void CollisionsToDistanceExpressions(const trajopt_interface::ContactResultVector& dist_results,
                                     planning_scene::PlanningSceneConstPtr planning_scene, std::string planning_group,
                                     const sco::VarVector& vars0, const sco::VarVector& vars1, const DblVec& x,
                                     sco::AffExprVector& exprs)
{
  sco::AffExprVector exprs0, exprs1;
  CollisionsToDistanceExpressions(dist_results, planning_scene, planning_group, vars0, x, exprs0, false);
  CollisionsToDistanceExpressions(dist_results, planning_scene, planning_group, vars1, x, exprs1, true);

  exprs.resize(exprs0.size());
  for (std::size_t i = 0; i < exprs0.size(); ++i)
  {
    // ??? dist_result: contact in MoveIt does not have cc_time
    assert(dist_results[i].cc_time >= 0.0 && dist_results[i].cc_time <= 1.0);
    sco::exprScale(exprs0[i], (1 - dist_results[i].cc_time));
    sco::exprScale(exprs1[i], dist_results[i].cc_time);
    exprs[i] = sco::AffExpr(0);
    sco::exprInc(exprs[i], exprs0[i]);
    sco::exprInc(exprs[i], exprs1[i]);
    sco::cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x)
{
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, trajopt_interface::ContactResultVector& dist_results)
{
  size_t key = hash(sco::getDblVec(x, GetVars()));
  trajopt_interface::ContactResultVector* it = m_cache.get(key);

  if (it != nullptr)
  {
    LOG_DEBUG("using cached collision check\n");
    dist_results = *it;
  }
  else
  {
    LOG_DEBUG("not using cached collision check\n");
    CalcCollisions(x, dist_results);
    m_cache.put(key, dist_results);
  }
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                   std::string planning_group,
                                                                   util::SafetyMarginData::ConstPtr safety_margin_data,
                                                                   const sco::VarVector& vars)
  : CollisionEvaluator(planning_scene, planning_group, safety_margin_data), m_vars(vars)
{
  // contact_manager_ = env_->getDiscreteContactManager();
  // contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  // contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
  //                                              0.04);  // The original implementation added a margin of 0.04;
}

// So, each pair has a vector of contacts, and this functions puts all of these vectors of all pairs to one big vector called dist_results
void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x,
                                                      trajopt_interface::ContactResultVector& dist_results)
{
  // tesseract::ContactResultMap contacts;
  // tesseract::EnvStatePtr state = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars));
  // ===> getVec is defined as:
  // Eigen::VectorXd getVec(const DblVec& x, const VarVector& vars) and
  // DblVec is vector<double>
  // VarVec is vector<Var>
  // Var has VarRep (look in solver_interface) member which has properties like name and index
  // getVec extracts the index of VarRep of each Var of a VarVector and finds the element in DblVec with the
  // same index and creates the output as VectorXd.
  // Finally, getState(joint_names, joint_values)
  // So, input x of this function, CalcCollisions, is represting the joint values
  // joint_values = sco::getVec(x, m_vars);

  // So, I guess I need to set the robot state based on these joint values
  Eigen::VectorXd joint_values = sco::getVec(x, m_vars);
  std::vector<double> joint_values_DblVec =
      std::vector<double>(joint_values.data(), joint_values.data() + joint_values.size());
  // the following robot state will be a copy not a pointer so it wont make any change in the planning scene
  moveit::core::RobotState& rob_state = planning_scene_->getCurrentStateNonConst();
  const moveit::core::JointModelGroup* joint_model_group = rob_state.getJointModelGroup(planning_group_);
  // rob_state.setToDefaultValues();
  rob_state.setJointGroupPositions(joint_model_group, joint_values_DblVec);
  rob_state.update();
  // Here is the point: I created a copy of the robot_state from planning_scene_->getCurrentState()
  // and updated that copy but that does not change anything in the planning_scene_
  // as it is a const pointer. const pointer planning scene comes from planning_context and
  // I can not change that.
  // planning_scene->setCurrentState(rob_state);
  std::vector<double> vv;
  planning_scene_->getCurrentState().copyJointGroupPositions(joint_model_group, vv);
  // trajopt::printVector("current state joint values: ", vv);

  // std::cout << "===>>> collision detection name: " << planning_scene_->getActiveCollisionDetectorName() << std::endl;
  // std::cout << "planning scene's name: " << planning_scene_->getName() << std::endl;

  // does planning_scene include the collision box?
  // std::vector<moveit_msgs::CollisionObject> collision_objs;
  // planning_scene_->getCollisionObjectMsgs(collision_objs);
  // for(std::size_t z = 0; z < collision_objs.size(); ++z)
  //   std::cout << "===>>> name of the collision object: " << collision_objs[z].id << " ";
  // std::cout << std::endl;

  // std::vector<std::string> obj_ids = planning_scene_->getWorld()->getObjectIds();
  // trajopt::printVector("world object ids: ", obj_ids);

  // std::cout << "===>>> robot model: " << std::endl;
  // std::cout << "name: " << planning_scene_->getRobotModel()->getName() << std::endl;
  // std::cout << "root joint: " <<  planning_scene_->getRobotModel()->getRootJointName() << std::endl;
  // std::cout << "root link: " <<  planning_scene_->getRobotModel()->getRootLinkName() << std::endl;
  // std::cout << "joint name: " <<  planning_scene_->getRobotModel()->getJointModelNames()[0] << std::endl;

  // std::vector<std::string> lnames = planning_scene_->getRobotModel()->getLinkModelNames();
  // trajopt::printVector("link model names: ", lnames);

  // for (const auto& link_name : manip_->getLinkNames())
  //   contact_manager_->setCollisionObjectsTransform(link_name, state->transforms[link_name]);
  // ====>
  // setCollisionObjectTransform sets the transform of a collision object.
  // contact manager has collision objects and has functions to remove/add them or
  // set their transforms. This is basically adding all the robot links as collision objects
  // We already have this in planning_scene, calling fcl or bullet would consider all the robot links as collision object?

  // contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  // ====> ContactTestTypes::ALL = > Return all contacts for a pair of objects

  // tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);
  // ====> moveContactResultsMapToContactResultsVector moves the elements of ContactResultVector
  // of each ContactResultMap in contacts to one ContactResultVector called dist_results

  // ===> for MoveIt
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.group_name = planning_group_;
  // collision_request.distance = true; // If true, compute proximity distance
  collision_request.contacts = true;  // If true, compute contacts. Otherwise only a binary collision yes/no is reported
  collision_request.max_contacts = 100;
  collision_request.max_contacts_per_pair = 5;
  collision_request.verbose = false;

  // planning_scene_->printKnownObjects();

  planning_scene_->checkCollision(collision_request, collision_result, rob_state);
  // ====> checkCollision will check for both self-collisions and for collisions with the environment
  LOG_DEBUG("single time step in collision? %s", collision_result.collision ? "true" : "false");
  LOG_DEBUG("contact count: %i", collision_result.contact_count);

  // tesseract:
  // ContactResultMap => AlignedMap<std::pair<std::string, std::string>, ContactResultVector>
  // ContactResultVector: is an array of ContactResult msg
  // ContactResult:
  //            distance
  //            array of two (link_names )
  // MoveIt:
  // CollisionResult:
  //          distance (min distance among all collision pairs) (double)
  //          contact_count (number of contacts)
  //          ContactMap
  //
  // ContactMap => std::map<std::pair<std::string, std::string>, std::vector<Contact> >
  // Contact:
  //          pos (contact position)
  //          normal (normal unit vector at contact)
  //          depth (penetration between bodies)
  //          body_name_1
  //          body_name_2
  //

  // Fill dist_result with all the Contacts of all the collision pairs. Each pair has a vector of contacts
  // and we have many pairs. Here we are putting all of these contact to one big vector. Not sure why that is ???
  // This happens in moveContactResultsMapToContactResultsVector
  for (auto it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());

    std::vector<collision_detection::Contact> contact_vector = it->second;
    for (size_t k = 0; k < contact_vector.size(); ++k)
    {
      // convert Contact from MoveIt to ContactResult
      Eigen::Vector3d moveit_cc_nearest_points[2];
      moveit_cc_nearest_points[0].setZero();
      moveit_cc_nearest_points[1].setZero();
      double moveit_cc_time = -1;
      trajopt_interface::ContinouseCollisionType moviet_cc_type =
          trajopt_interface::ContinouseCollisionType::CCType_None;

      ContactResult contact_result(contact_vector[k], moveit_cc_nearest_points, moveit_cc_time, moviet_cc_type);
      dist_results.push_back(contact_result);
    }
  }
}

// so, given a set of joint values called x, this function converts the vector of contacts to the vector of distances
void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  trajopt_interface::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  trajopt_interface::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, planning_scene_, planning_group_, m_vars, x, exprs, false);
  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(planning_scene::PlanningSceneConstPtr planning_scene,
                                               std::string planning_group,
                                               util::SafetyMarginData::ConstPtr safety_margin_data,
                                               const sco::VarVector& vars0, const sco::VarVector& vars1)
  : CollisionEvaluator(planning_scene, planning_group, safety_margin_data), m_vars0(vars0), m_vars1(vars1)
{
  // contact_manager_ = env_->getContinuousContactManager();
  // contact_manager_->setActiveCollisionObjects(manip_->getLinkNames());
  // contact_manager_->setContactDistanceThreshold(safety_margin_data_->getMaxSafetyMargin() +
  //                                               0.04);  // The original implementation added a margin of 0.04;
}

// similar to CalcCollisions from SingleTimestepCollisionEvaluator except we need two states because it is the
// swept volume?
void CastCollisionEvaluator::CalcCollisions(const DblVec& x, trajopt_interface::ContactResultVector& dist_results)
{
  // tesseract::ContactResultMap contacts;
  // tesseract::EnvStatePtr state0 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars0));
  // tesseract::EnvStatePtr state1 = env_->getState(manip_->getJointNames(), sco::getVec(x, m_vars1));
  // ===> so far they create two states based on two sets of joint values

  // for (const auto& link_name : manip_->getLinkNames())
  //   contact_manager_->setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);
  // ===>>> in here, they add two collision transforms for each link

  // ===>>> in the folloinwg, considering two collision transforms for each link, they calculate all the contacts among
  // them contact_manager_->contactTest(contacts, tesseract::ContactTestTypes::ALL);
  // tesseract::moveContactResultsMapToContactResultsVector(contacts, dist_results);

  // In MoveIt we do have a checkCollision function that gets the two states in continous manner
  // when using Bullet that function will be overriden by a real continous collision checking process
  // All I have to do here is use that function for two

  // =====>>> So, I need to do it for two states as well.
  // x has joint valus for all time steps: [j1_0 j2_0 ... j7_0, j1_1 j2_1 ... j7_1 ... j1_n j2_n ... j7_n]
  Eigen::VectorXd joint_values_state_0 = sco::getVec(x, m_vars0);
  std::vector<double> joint_values_DblVec_0 =
      std::vector<double>(joint_values_state_0.data(), joint_values_state_0.data() + joint_values_state_0.size());

  moveit::core::RobotState& rob_state = planning_scene_->getCurrentStateNonConst();
  const moveit::core::JointModelGroup* joint_model_group = rob_state.getJointModelGroup(planning_group_);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.group_name = planning_group_;
  // collision_request.distance = true;
  collision_request.contacts = true;
  collision_request.max_contacts = 100;
  collision_request.max_contacts_per_pair = 5;
  collision_request.verbose = false;

  rob_state.setToDefaultValues();
  rob_state.setJointGroupPositions(joint_model_group, joint_values_DblVec_0);
  rob_state.update();
  robot_state::RobotState rob_state_previous(rob_state);

  // Eigen::VectorXd first_values;
  // rob_state.copyJointGroupPositions(joint_model_group, first_values);
  // std::cout << "===>>> first state joint values: " << first_values.transpose() << std::endl;

  Eigen::VectorXd joint_values_state_1 = sco::getVec(x, m_vars1);
  std::vector<double> joint_values_DblVec_1 =
      std::vector<double>(joint_values_state_1.data(), joint_values_state_1.data() + joint_values_state_1.size());
  rob_state.setJointGroupPositions(joint_model_group, joint_values_DblVec_1);
  rob_state.update();

  // Eigen::VectorXd second_values;
  // rob_state.copyJointGroupPositions(joint_model_group, second_values);
  // std::cout << "===>>> second state joint values: " << second_values.transpose() << std::endl;

  // check collision between two states
  planning_scene_->getCollisionEnv()->checkRobotCollision(collision_request, collision_result, rob_state,
                                                          rob_state_previous);

  LOG_DEBUG("single time step in collision? %s", collision_result.collision ? "true" : "false");
  LOG_DEBUG("contact count: %i", collision_result.contact_count);

  // ????? I have to calculate: cc_time, cc_type and cc_nearest_points here. test

  // when we use bullet in MoveIt, CollisionResult returns all the contacts in the scene by ContactMap data type.
  // ContactMap < <link_1, link_2>, vector<contact>>
  // when using bullet, vector<Contact> has only one member while fcl might return multiple contact points for
  // each pair in contact

  for (auto it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());

    // using bullet returns one contact point per pair of contact objects but just in case, I traverse through
    // all the element of the vector<collision_detection::Contact>
    std::vector<collision_detection::Contact> contact_vector = it->second;
    for (size_t k = 0; k < contact_vector.size(); ++k)
    {
      // convert Contact from MoveIt to ContactResult in tesseract:
      Eigen::Vector3d moveit_cc_nearest_points[2];
      moveit_cc_nearest_points[0] = contact_vector[k].nearest_points[0];
      moveit_cc_nearest_points[1] = contact_vector[k].nearest_points[1];
      ;
      double moveit_cc_time = 0.5;
      trajopt_interface::ContinouseCollisionType moviet_cc_type =
          trajopt_interface::ContinouseCollisionType::CCType_Between;

      ContactResult contact_result(contact_vector[k], moveit_cc_nearest_points, moveit_cc_time, moviet_cc_type);
      dist_results.push_back(contact_result);
    }
  }
}

void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs)
{
  trajopt_interface::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistanceExpressions(dist_results, planning_scene_, planning_group_, m_vars0, m_vars1, x, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists)
{
  trajopt_interface::ContactResultVector dist_results;
  GetCollisionsCached(x, dist_results);
  CollisionsToDistances(dist_results, dists);
}

//////////////////////////////////////////

CollisionCost::CollisionCost(planning_scene::PlanningSceneConstPtr& planning_scene, std::string planning_group,
                             util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars)
  : Cost("collision")
  , m_calc(new SingleTimestepCollisionEvaluator(planning_scene, planning_group, safety_margin_data, vars))
{
}

CollisionCost::CollisionCost(planning_scene::PlanningSceneConstPtr& planning_scene, std::string planning_group,
                             util::SafetyMarginData::ConstPtr safety_margin_data, const sco::VarVector& vars0,
                             const sco::VarVector& vars1)
  : Cost("cast_collision")
  , m_calc(new CastCollisionEvaluator(planning_scene, planning_group, safety_margin_data, vars0, vars1))
{
}

sco::ConvexObjective::Ptr CollisionCost::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexObjective::Ptr out(new sco::ConvexObjective(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  trajopt_interface::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
    out->addHinge(viol, data[1]);
  }
  return out;
}

double CollisionCost::value(const sco::DblVec& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  trajopt_interface::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  double out = 0;
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    //??? link_name from dist_results in MoveIt
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);
    out += sco::pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}

// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(planning_scene::PlanningSceneConstPtr planning_scene,
                                         std::string planning_group,
                                         util::SafetyMarginData::ConstPtr safety_margin_data,
                                         const sco::VarVector& vars)
  : m_calc(new SingleTimestepCollisionEvaluator(planning_scene, planning_group, safety_margin_data, vars))
{
  name_ = "collision";
}

CollisionConstraint::CollisionConstraint(planning_scene::PlanningSceneConstPtr planning_scene,
                                         std::string planning_group,
                                         util::SafetyMarginData::ConstPtr safety_margin_data,
                                         const sco::VarVector& vars0, const sco::VarVector& vars1)
  : m_calc(new CastCollisionEvaluator(planning_scene, planning_group, safety_margin_data, vars0, vars1))
{
  name_ = "collision";
}

sco::ConvexConstraints::Ptr CollisionConstraint::convex(const sco::DblVec& x, sco::Model* model)
{
  sco::ConvexConstraints::Ptr out(new sco::ConvexConstraints(model));
  sco::AffExprVector exprs;
  m_calc->CalcDistExpressions(x, exprs);

  trajopt_interface::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  for (std::size_t i = 0; i < exprs.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    sco::AffExpr viol = sco::exprSub(sco::AffExpr(data[0]), exprs[i]);
    out->addIneqCnt(sco::exprMult(viol, data[1]));
  }
  return out;
}

DblVec CollisionConstraint::value(const sco::DblVec& x)
{
  DblVec dists;
  m_calc->CalcDists(x, dists);

  trajopt_interface::ContactResultVector dist_results;
  m_calc->GetCollisionsCached(x, dist_results);
  DblVec out(dists.size());
  for (std::size_t i = 0; i < dists.size(); ++i)
  {
    const Eigen::Vector2d& data = m_calc->getSafetyMarginData()->getPairSafetyMarginData(dist_results[i].link_names[0],
                                                                                         dist_results[i].link_names[1]);

    out[i] = sco::pospart(data[0] - dists[i]) * data[1];
  }
  return out;
}
}  // namespace trajopt_interface
